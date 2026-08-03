# -*- coding: utf-8 -*-
"""本机 USB-CAN 串口直连后端（CanBackend）。

机械臂经 USB 转 CAN 模块（枚举为串口，Windows=COM4 / Linux=/dev/ttyACM0）通信，
厂商库 DrEmpower.py 把 CAN 报文封装为 16 字节串口帧。

线程模型
--------
* 厂商库所有函数共享模块级全局（uart / READ_FLAG 等），**绝不允许并发调用**；
  因此全部 dr.* 调用收敛到唯一的工作线程（self._thread）中执行。
* GUI 线程只做：参数校验 + 往 queue.Queue 投命令 + 置事件标志；全部非阻塞。
* 状态 / 日志 / 连接变化通过 Qt 信号发回（信号可跨线程安全 emit）。

安全约定（与 backend_base 一致）
------------------------------
* 运动默认禁止；command_angles / set_zero_here 未使能时拒绝并日志。
* estop() 任何状态下可用：置事件标志，工作线程在每个循环点最优先检查并
  立即广播 dr.estop(0)，同时清空待发命令队列。
* 目标角度经 validate_targets() 软限位/步长校验，任一超限整条拒绝。
"""
from __future__ import annotations

import math
import os
import queue
import struct
import threading
import time
from typing import Dict, List, Optional, Tuple

import serial

# 厂商库怪癖：模块在 import 时就会按环境变量 DREMPOWER_PORT 打开串口。
# 先设为必然打开失败的假端口（库内仅打印一行并置 uart=None，无害），
# 运行时（工作线程内）再给 dr.uart 赋真正的 serial.Serial 对象。
os.environ['DREMPOWER_PORT'] = 'DISABLED_AT_IMPORT'
from ..vendor.drempower import DrEmpower as dr  # noqa: E402  (须在设置环境变量后导入)

from .backend_base import ArmBackend, make_offline_state  # noqa: E402
from .state import ArmState, JointReading  # noqa: E402
from ..i18n import tr, register  # noqa: E402


class CanBackend(ArmBackend):
    """USB-CAN 串口直连后端。

    start(params) 的 params 字段：
        port         str    串口名，如 'COM4' / '/dev/ttyACM0'
        baud         int    串口波特率，默认 115200
        joint_ids    int[]  关节 CAN ID 列表，如 [1, 2, 3, 4]
        joint_names  str[]  关节名列表（长度须与 joint_ids 一致，否则自动生成）
        poll_hz      float  状态轮询频率，默认 20.0
    """

    name = 'can'

    # 连续多少轮状态读取全部失败后判定连接丢失（每轮失败约耗时 1 个读超时）
    _DISCONNECT_AFTER_FAILS = 5
    # 状态帧突发读取超时：4 帧正常 <10ms 到齐，0.25s 富余且不拖垮轮询节奏
    _STATE_READ_TIMEOUT_S = 0.25
    # 单关节缺帧去抖：连续缺帧超过该轮数才标离线（偶发丢 1 帧不闪烁）
    _OFFLINE_AFTER_MISSES = 2
    # 电压/电流补充读取周期（秒），约 1Hz
    _VOL_CUR_PERIOD_S = 1.0
    # 同类告警日志的最小间隔（秒），避免 20Hz 失败刷屏
    _LOG_THROTTLE_S = 5.0
    # 现场固件使用 ODrive 状态枚举：1=IDLE，8=CLOSED_LOOP_CONTROL。
    # requested_state=2 不是闭环，承重关节会退出保持，严禁写入。
    _IDLE_STATE = 1
    _CLOSED_LOOP_STATE = 8
    _MODE_VERIFY_SETTLE_S = 0.15

    def __init__(self, parent=None):
        super().__init__(parent)
        # ---- 连接参数（start 时填充）----
        self._port = ''
        self._baud = 115200
        self._joint_ids: List[int] = []
        self._joint_names: List[str] = []
        self._poll_hz = 20.0
        self._gripper_id = 0            # 夹爪电机 CAN ID（0 = 未配置）
        self._gripper_speed = 10.0
        self._gripper_accel = 20.0

        # ---- 线程与同步原语 ----
        # 命令队列：GUI 线程 put，工作线程 get（queue.Queue 自带锁，线程安全）
        self._cmd_queue: 'queue.Queue[tuple]' = queue.Queue()
        self._stop_event = threading.Event()    # shutdown / 致命断连 -> 停止工作线程
        self._estop_event = threading.Event()   # 急停请求，工作线程最优先处理
        self._wake_event = threading.Event()    # 唤醒轮询间隔中的等待（新命令/急停/关闭）
        self._thread: Optional[threading.Thread] = None

        # ---- 状态缓存 ----
        # bool/引用赋值在 CPython(GIL) 下原子，GUI 线程读、工作线程写，无需额外锁
        self._connected = False
        self._last_state: Optional[ArmState] = None
        self._closed_loop_verified = False
        self._drive_errors: Dict[int, Dict[str, Optional[int]]] = {}
        # 本后端实例当前持有的串口（仅工作线程赋值/清空；shutdown 兜底关闭时
        # 用它做身份判断，防止误关/误清其他实例赋给 dr.uart 的新串口）
        self._ser: Optional[serial.Serial] = None
        # 工作线程内部使用：同类日志的上次发出时刻（限流）
        self._log_ts: Dict[str, float] = {}

    # ================= 连接管理（GUI 线程调用，非阻塞） =================
    def start(self, params: dict) -> None:
        """异步发起连接：解析参数并启动工作线程，本方法立即返回。"""
        if self._thread is not None and self._thread.is_alive():
            self.log_message.emit('warn', tr('CAN backend already running, ignoring duplicate start()'))
            return

        self._port = str(params.get('port', 'COM4'))
        self._baud = int(params.get('baud', 115200))
        self._joint_ids = [int(x) for x in params.get('joint_ids', [1, 2, 3, 4])]
        names = [str(x) for x in params.get('joint_names', [])]
        if len(names) != len(self._joint_ids):
            names = ['joint%d' % jid for jid in self._joint_ids]
        self._joint_names = names
        self._poll_hz = float(params.get('poll_hz', 20.0))
        self._gripper_id = int(params.get('gripper_id', 0))
        self._gripper_speed = float(params.get('gripper_speed_rpm', 10.0))
        self._gripper_accel = float(params.get('gripper_accel', 20.0))

        # 复位同步原语，允许 shutdown 后再次 start
        self._stop_event.clear()
        self._estop_event.clear()
        self._wake_event.clear()
        self._drain_queue()
        self._log_ts.clear()
        # 兜底：上次会话的使能状态不得带入新连接（运动默认禁止）
        self._closed_loop_verified = False
        self._drive_errors = {}
        if self._motion_enabled:
            self._motion_enabled = False
            self.motion_enabled_changed.emit(False)
        self._last_state = make_offline_state(self._joint_ids, self._joint_names, self.name)

        self._thread = threading.Thread(target=self._worker,
                                        name='can-backend-worker', daemon=True)
        self._thread.start()
        self.log_message.emit('info', tr('Connecting serial port %s @ %d ...') % (self._port, self._baud))

    def shutdown(self) -> None:
        """断开并回收线程（幂等，允许短暂阻塞）。串口由工作线程在退出前关闭。"""
        self._stop_event.set()
        self._wake_event.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=3.0)
            if t.is_alive():
                # 线程可能仍阻塞在 uart.read()：不强行关串口（避免并发读/关句柄），
                # daemon 线程与串口句柄交由进程退出回收；残留线程退出时只会
                # 关闭它自己的串口（见 _worker finally 的身份判断）
                self.log_message.emit('error', tr('Worker thread did not exit within 3s; serial port left to process-exit cleanup'))
                self._connected = False
                return
        self._thread = None
        # 兜底：正常情况下串口已在工作线程 finally 中关闭并置 None。
        # 只回收本实例自己的串口，且仅当 dr.uart 仍指向它时才置空，
        # 防止误关/误清其他实例（新连接）的串口
        ser = self._ser
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
            if getattr(dr, 'uart', None) is ser:
                dr.uart = None
            self._ser = None
        self._connected = False

    def is_connected(self) -> bool:
        return bool(self._connected)

    # ================= 运动使能（GUI 线程调用） =================
    def set_motion_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled and not self.is_connected():
            # 与 SshBackend 对齐：未连接时不允许打开使能门（关闭方向永远放行）
            self.log_message.emit('warn', tr('Enable rejected: backend not connected'))
            return
        if enabled and not self._closed_loop_verified:
            self.log_message.emit(
                'error', tr('Enable rejected: J1-J4 closed-loop state was not verified'))
            return
        if enabled == self._motion_enabled:
            return
        self._motion_enabled = enabled
        if not enabled:
            # 关闭使能时同时丢弃全部待发命令，防止旧命令在重新使能后被意外执行
            self._drain_queue()
        self.motion_enabled_changed.emit(enabled)
        if enabled:
            self.log_message.emit('warn', tr('Motion enable is ON: commands will drive real joints!'))
        else:
            self.log_message.emit('info', tr('Motion enable is OFF; pending command queue cleared'))

    # ================= 控制命令（GUI 线程调用，全部异步入队） =================
    def command_angles(self, angles_deg: Dict[int, float],
                       speed_rpm: float, accel: float,
                       allow_large_step: bool = False) -> None:
        """多关节绝对角度命令（梯形轨迹 mode=1）。校验通过后入队，由工作线程下发。

        allow_large_step=True：跳过步长校验（软限位仍硬性拦截），
        供 UI 在用户明确确认大幅移动后调用。
        """
        if not self.is_connected():
            self.log_message.emit('warn', tr('Angle command rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit('warn', tr('Angle command rejected: motion not enabled (enable motion first)'))
            return
        if not angles_deg:
            self.log_message.emit('warn', tr('Angle command rejected: empty targets'))
            return
        unknown = [jid for jid in angles_deg if jid not in self._joint_ids]
        if unknown:
            self.log_message.emit('error', tr('Angle command rejected: unknown joint ID %s') % unknown)
            return
        # 软限位（硬性）+ 单次步长校验（确认过的大幅移动可跳过）
        ok, reason = self.validate_targets(angles_deg, self._last_state,
                                           check_step=not allow_large_step)
        if not ok:
            self.log_message.emit('error', tr('Angle command rejected: %s') % reason)
            return
        speed, acc = self.clamp_motion_params(speed_rpm, accel)
        # 按配置顺序组装 id/angle 列表（允许只命令部分关节）
        id_list = [jid for jid in self._joint_ids if jid in angles_deg]
        angle_list = [float(angles_deg[jid]) for jid in id_list]
        self._cmd_queue.put(('angles', id_list, angle_list, speed, acc))
        self._wake_event.set()
        self.log_message.emit('info', tr('Target angles queued: %s (speed=%.1f r/min, accel=%.1f)')
                              % ({j: round(a, 2) for j, a in zip(id_list, angle_list)},
                                 speed, acc))

    def estop(self) -> None:
        """急停（广播）。任何状态下可调用（包括未使能），优先于队列中其他命令。"""
        self._estop_event.set()   # 工作线程每个循环点最优先检查该标志
        self._drain_queue()       # 立即丢弃全部待发命令
        self._wake_event.set()    # 打断轮询间隔中的等待，尽快发出
        # 急停后自动关闭使能门（与 SshBackend / remote_agent 语义一致）：
        # 需用户重新确认使能才能继续运动
        if self._motion_enabled:
            self._motion_enabled = False
            self.motion_enabled_changed.emit(False)
            self.log_message.emit('warn', tr('Motion enable auto-disabled after E-stop; re-enable to move again'))
        if self._thread is not None and self._thread.is_alive():
            self.log_message.emit('warn', tr('!!! E-stop request submitted (command queue cleared) !!!'))
        else:
            self.log_message.emit('error', tr('E-stop request: backend not connected, cannot send to bus!'))

    def set_zero_here(self) -> None:
        """将当前位置设为零点（广播，会短暂卸载关节，需使能运动）。"""
        if not self.is_connected():
            self.log_message.emit('warn', tr('Set-zero rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit('warn', tr('Set-zero rejected: motion not enabled (this operation briefly unloads joints)'))
            return
        self._cmd_queue.put(('zero',))
        self._wake_event.set()
        self.log_message.emit('warn', tr('Set-zero command queued (broadcast; joints will unload briefly)'))

    def set_poll_rate(self, hz: float) -> None:
        self._poll_hz = max(0.5, min(float(hz), 100.0))  # float 赋值原子，工作线程每轮读取
        self._wake_event.set()
        self.log_message.emit('info', tr('Poll rate set to %.1f Hz') % self._poll_hz)

    def gripper_command(self, angle_deg: float,
                        speed_rpm: Optional[float] = None,
                        accel: Optional[float] = None) -> None:
        """夹爪电机绝对角度命令（梯形轨迹，需使能运动）。"""
        if not self._gripper_id:
            self.log_message.emit('warn', tr('Gripper not configured (gripper.motor_id=0)'))
            return
        if not self.is_connected():
            self.log_message.emit('warn', tr('Gripper command rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit('warn', tr('Gripper command rejected: motion not enabled'))
            return
        speed = self._gripper_speed if speed_rpm is None else float(speed_rpm)
        acc = self._gripper_accel if accel is None else float(accel)
        self._cmd_queue.put(('gripper', float(angle_deg), speed, acc))
        self._wake_event.set()

    def gripper_command_adaptive(self, angle_deg: float, speed_rpm: float,
                                 torque_nm: float) -> None:
        """Torque-limited absolute gripper motion.

        This is intended for gentle object closure.  It is deliberately a
        gripper-only command and never changes the mode of J1--J4.
        """
        if not self._gripper_id:
            self.log_message.emit(
                'warn', tr('Gripper not configured (gripper.motor_id=0)'))
            return
        if not self.is_connected():
            self.log_message.emit(
                'warn', tr('Gripper command rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit(
                'warn', tr('Gripper command rejected: motion not enabled'))
            return
        if speed_rpm <= 0.0 or torque_nm <= 0.0:
            self.log_message.emit(
                'error', 'Adaptive gripper speed and torque must be positive')
            return
        self._cmd_queue.put(('gripper_adaptive', float(angle_deg),
                             float(speed_rpm), float(torque_nm)))
        self._wake_event.set()

    def gripper_set_zero(self) -> None:
        """把夹爪当前位置设为闭合零点（仅夹爪电机，掉电保存，需使能运动）。"""
        if not self._gripper_id:
            self.log_message.emit('warn', tr('Gripper not configured (gripper.motor_id=0)'))
            return
        if not self.is_connected():
            self.log_message.emit('warn', tr('Gripper zero rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit('warn', tr('Gripper zero rejected: motion not enabled'))
            return
        self._cmd_queue.put(('gripper_zero',))
        self._wake_event.set()

    def set_joints_mode(self, mode: int) -> None:
        """广播切换关节模式：1=待机(卸载/可手推)，2=闭环(锁定保持)。

        释放(1)后关节失去保持力矩（可手动摆位），同时自动关闭运动使能门；
        厂商约定运动控制必须在闭环模式下执行，故释放后需先「锁定」再使能运动。
        """
        if mode not in (1, 2):
            self.log_message.emit('error', tr('Unknown joint mode %s (1=standby/release, 2=closed-loop/lock)') % mode)
            return
        if not self.is_connected():
            self.log_message.emit('warn', tr('Joint mode switch rejected: backend not connected'))
            return
        if self._estop_event.is_set():
            self.log_message.emit(
                'warn', tr('Joint mode switch rejected: E-stop is still being processed'))
            return
        if mode == 1:
            # 释放 = 脱离软件控制：关使能门并清空待发命令（同急停语义）
            self._closed_loop_verified = False
            self._drain_queue()
            if self._motion_enabled:
                self._motion_enabled = False
                self.motion_enabled_changed.emit(False)
                self.log_message.emit('warn', tr('Releasing joints: motion enable auto-disabled'))
        self._cmd_queue.put(('mode', int(mode)))
        self._wake_event.set()

    # ================= 工作线程（唯一允许调用 dr.* 的地方） =================
    def _worker(self) -> None:
        """工作线程主体：打开串口 -> 扫描总线 -> 轮询循环 -> 清理退出。"""
        connected_once = False
        disconnect_reason = tr('Disconnected')
        ser: Optional[serial.Serial] = None
        try:
            # ---- 1. 打开串口并交给厂商库 ----
            try:
                # write_timeout 兜底：设备停摆时 uart.write 不会无限阻塞，
                # 降低 shutdown() join 超时的概率
                ser = serial.Serial(self._port, self._baud,
                                    timeout=1.0, write_timeout=1.0)
            except Exception as e:
                self.log_message.emit('error', tr('Failed to open serial port %s: %s') % (self._port, e))
                self.connection_changed.emit(False, tr('Failed to open serial port %s: %s') % (self._port, e))
                return
            self._ser = ser
            dr.uart = ser

            # ---- 2. 清接收缓冲（clear_uart 自身不捕获异常，须在此包住）----
            try:
                dr.clear_uart()
            except Exception as e:
                self.log_message.emit('warn', tr('Failed to clear serial receive buffer: %s') % e)

            # ---- 3. 总线扫描（失败不致命，仅日志提示）----
            try:
                ids = dr.get_ids()
                if ids:
                    self.log_message.emit('info', tr('Bus scan found joint IDs: %s') % ids)
                else:
                    self.log_message.emit('warn', tr('Bus scan got no joint response (non-fatal; check joint power/CAN wiring, polling continues)'))
            except Exception as e:
                self.log_message.emit('warn', tr('Bus scan error (non-fatal): %s') % e)

            # ---- 3.5 逐轴核验真实状态（现场固件：1=待机，8=闭环）----
            states = self._read_joint_states()
            self._drive_errors = self._read_joint_errors()
            all_closed = (len(states) == len(self._joint_ids)
                          and all(v == self._CLOSED_LOOP_STATE
                                  for v in states.values()))
            errors_readable = all(
                all(v is not None for v in per_joint.values())
                for per_joint in self._drive_errors.values())
            errors_clear = (len(self._drive_errors) == len(self._joint_ids)
                            and errors_readable
                            and all(v == 0
                                    for per_joint in self._drive_errors.values()
                                    for v in per_joint.values()))
            all_idle = (len(states) == len(self._joint_ids)
                        and all(v == self._IDLE_STATE for v in states.values()))
            self._closed_loop_verified = all_closed and errors_clear
            init_mode = 2 if all_closed else (1 if all_idle else 0)
            self.joints_mode_changed.emit(init_mode)
            if not all_closed:
                self.log_message.emit(
                    'error', tr('J1-J4 closed-loop verification failed: %s') % states)
            if not errors_clear:
                self.log_message.emit(
                    'error', tr('J1-J4 drive error preflight failed: %s')
                    % self._drive_errors)

            # ---- 4. 宣告连接成功，进入轮询循环 ----
            self._connected = True
            connected_once = True
            self.connection_changed.emit(True, tr('Connected %s @ %d') % (self._port, self._baud))
            disconnect_reason = self._run_loop()
        finally:
            # 串口由本线程独占，也只在本线程内关闭，避免与阻塞读并发。
            # 只回收本线程自己打开的 ser；dr.uart 仅在仍指向 ser 时才置空——
            # 若本线程曾卡死超过 shutdown() 的 join 超时、GUI 已另起新连接，
            # dr.uart 已指向新串口，此处绝不能关掉/清空它
            self._connected = False
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                if getattr(dr, 'uart', None) is ser:
                    dr.uart = None
                if self._ser is ser:
                    self._ser = None
            self._drain_queue()
            # 断连即复位使能门：同一实例再次 start() 不得带着旧使能状态
            self._closed_loop_verified = False
            try:
                if self._motion_enabled:
                    self._motion_enabled = False
                    self.motion_enabled_changed.emit(False)
                if connected_once:
                    self.connection_changed.emit(False, disconnect_reason)
            except RuntimeError:
                pass   # 残留线程场景：后端 QObject 可能已被 GUI 线程 deleteLater

    def _run_loop(self) -> str:
        """轮询主循环。返回断开原因（供 connection_changed 使用）。

        每轮顺序：急停检查 -> 处理队列中全部命令 -> 状态轮询 ->
                  ~1Hz 电压电流 -> 发布 ArmState -> 控频等待。
        """
        consec_fails = 0          # 连续全失败轮数（用于断连判定）
        last_vc_t = 0.0           # 上次读电压/电流的时刻
        bus_v = math.nan          # 最近一次读到的母线电压 / 电流（读取间隔内保持）
        bus_c = math.nan
        vc_joint: Optional[int] = None   # 用于读电压电流的在线关节 ID
        miss_counts: Dict[int, int] = {}                              # 每关节连续缺帧计数
        last_frames: Dict[int, Tuple[float, float, float]] = {}       # 每关节最近一帧

        while True:
            if self._stop_event.is_set():
                return tr('Disconnected')
            t_loop = time.monotonic()

            # ---- 急停最优先，其次处理完队列中全部命令 ----
            self._service_estop()
            self._process_commands()
            if self._stop_event.is_set():
                return tr('Disconnected')

            # ---- 状态轮询（latency 统计本轮全部串口耗时）----
            t_io = time.monotonic()
            frames: Dict[int, Tuple[float, float, float]] = {}
            try:
                frames = self._poll_states()
            except Exception as e:   # 双保险：含 TimeoutError / 串口异常
                self._log_throttled('warn', 'poll', tr('Error reading joint states: %s') % e)

            # ---- 单帧丢失去抖：偶发缺帧沿用上一帧读数（≤_OFFLINE_AFTER_MISSES 轮）----
            all_ids = (list(self._joint_ids)
                       + ([self._gripper_id] if self._gripper_id else []))
            for jid in all_ids:
                if jid in frames:
                    miss_counts[jid] = 0
                    last_frames[jid] = frames[jid]
                else:
                    miss_counts[jid] = miss_counts.get(jid, 999) + 1
                    if miss_counts[jid] <= self._OFFLINE_AFTER_MISSES and jid in last_frames:
                        frames[jid] = last_frames[jid]

            joints, first_online = self._assemble_joints(frames)

            # ---- 夹爪读数（独立字段，不进关节卡片） ----
            greading = None
            if self._gripper_id:
                entry = frames.get(self._gripper_id)
                if entry is not None:
                    greading = JointReading(joint_id=self._gripper_id, name='gripper',
                                            angle_deg=float(entry[0]),
                                            speed_rpm=float(entry[1]),
                                            torque_nm=float(entry[2]), online=True)
                else:
                    greading = JointReading(joint_id=self._gripper_id,
                                            name='gripper', online=False)

            # ---- ~1Hz 用第一个在线关节补充电压/电流 ----
            if first_online is not None:
                vc_joint = first_online
            now = time.monotonic()
            if vc_joint is not None and now - last_vc_t >= self._VOL_CUR_PERIOD_S:
                last_vc_t = now
                try:
                    # 整体失败返回 False；部分失败返回 [False, cur] 等（read_property
                    # 失败项为 False）。bool 是 int 子类，须显式排除，
                    # 否则 float(False)=0.0 会被误显示成 0.0V/0.0A（貌似掉电）
                    vc = dr.get_vol_cur(vc_joint)
                    if (isinstance(vc, (list, tuple)) and len(vc) == 2
                            and all(isinstance(x, (int, float))
                                    and not isinstance(x, bool) for x in vc)):
                        bus_v = float(vc[0])
                        bus_c = float(vc[1])
                except Exception as e:
                    self._log_throttled('warn', 'volcur', tr('Error reading voltage/current: %s') % e)
            latency_ms = (time.monotonic() - t_io) * 1000.0

            # ---- 发布状态快照（失败轮也发布，UI 可显示离线）----
            state = ArmState(joints=joints, bus_voltage=bus_v, bus_current=bus_c,
                             latency_ms=latency_ms, timestamp=time.time(),
                             source=self.name, gripper=greading)
            self._last_state = state
            self.state_updated.emit(state)

            # ---- 断连判定：连续 N 轮无任何关节响应 ----
            if any(j.online for j in joints):
                consec_fails = 0
            else:
                consec_fails += 1
                self._log_throttled('warn', 'allfail',
                                    tr('No joint responded this cycle (%d/%d in a row)')
                                    % (consec_fails, self._DISCONNECT_AFTER_FAILS))
                if consec_fails >= self._DISCONNECT_AFTER_FAILS:
                    return (tr('All reads timed out/failed for %d consecutive cycles; connection lost')
                            % consec_fails)

            # ---- 控频等待（新命令/急停/关闭会立即唤醒）----
            period = 1.0 / self._poll_hz if self._poll_hz > 0 else 0.05
            remain = period - (time.monotonic() - t_loop)
            if remain > 0:
                self._wake_event.wait(remain)
            self._wake_event.clear()

    def _poll_states(self) -> Dict[int, Tuple[float, float, float]]:
        """健壮版状态轮询（替代厂商 get_angle_speed_torque_all，规避其组帧竞态）。

        厂商 read_data_state2 依赖 inWaiting() 判定读取结束：4 个关节的应答帧
        到达存在毫秒级时间差（Windows 的 USB 串口按 ~1ms 批次交付字节），
        Python 读到缓冲区暂空即提前收工 → 少帧则整轮失败；失败后迟到的帧残留
        在缓冲区又污染下一轮解析 → 时好时坏（UI 闪烁）。
        此处改为：清残留 → 广播一次性状态请求(0x08/order 0x24，与厂商一致)
        → 按超时整块读取 → 0xAA 帧头滑窗解析 + 去重取最新。
        返回 {关节ID: (angle_deg, speed_rpm, torque_nm)}；缺帧关节不在字典中
        （由 _assemble_joints 标记离线，支持部分在线显示）。
        """
        ser = self._ser
        if ser is None:
            return {}
        ser.reset_input_buffer()      # 丢弃上一轮迟到/残留字节，防止陈旧帧错位
        dr.send_command(id_num=0, cmd=0x08,
                        data=dr.format_data([0x24], 'u32', 'encode'))
        # The broadcast also returns the configured gripper frame.  Reading
        # only J1-J4 bytes can leave that frame in the UART buffer and pollute
        # the next property/command transaction.
        n = max(1, len(self._joint_ids)
                + (1 if self._gripper_id is not None else 0))
        old_timeout = ser.timeout
        try:
            ser.timeout = self._STATE_READ_TIMEOUT_S
            raw = ser.read(16 * n)    # 阻塞至读满或超时（正常 <10ms 到齐）
            extra = ser.in_waiting    # 同批已到的尾部字节一并取走参与解析
            if extra:
                raw += ser.read(extra)
        finally:
            ser.timeout = old_timeout

        out: Dict[int, Tuple[float, float, float]] = {}
        i = 0
        while i + 16 <= len(raw):
            # 16 字节帧：AA IDE RTR DLC(0x08) 00 00 idH idL d0..d7
            if raw[i] == 0xAA and raw[i + 3] == 0x08:
                frame = raw[i:i + 16]
                jid = (((frame[6] << 8) | frame[7]) - 1) >> 5   # 同厂商 uart_to_can_ID
                try:
                    angle, spd, tq = struct.unpack('<fhh', bytes(frame[8:16]))
                except struct.error:
                    i += 1
                    continue
                if ((jid in self._joint_ids or jid == self._gripper_id)
                        and math.isfinite(angle)):
                    out[jid] = (angle, spd * 0.01, tq * 0.01)   # 重复帧取最新
                i += 16
            else:
                i += 1                # 滑窗重同步（跳过错位/噪声字节）
        return out

    def _assemble_joints(self, frames: Dict[int, Tuple[float, float, float]]
                         ) -> 'tuple[List[JointReading], Optional[int]]':
        """把 _poll_states 的结果组装为 JointReading 列表（按配置顺序）。

        返回 (joints, 第一个在线关节 ID 或 None)。
        """
        joints: List[JointReading] = []
        first_online: Optional[int] = None
        for i, jid in enumerate(self._joint_ids):
            name = self._joint_names[i] if i < len(self._joint_names) else 'joint%d' % jid
            entry = frames.get(jid)
            if entry is not None:
                joints.append(JointReading(joint_id=jid, name=name,
                                           angle_deg=float(entry[0]),
                                           speed_rpm=float(entry[1]),
                                           torque_nm=float(entry[2]),
                                           online=True))
                if first_online is None:
                    first_online = jid
            else:
                joints.append(JointReading(joint_id=jid, name=name, online=False))
        return joints, first_online

    def _service_estop(self) -> None:
        """急停检查点：三段式安全停止（不看 motion_enabled）并清空队列。

        仅发厂商 estop 可能使关节卸载 -> 重力下机械臂坠落砸桌！
        安全序列：①厂商急停终止当前轨迹 ②立即重读实际位置
                  ③广播闭环模式 + 把目标角锁定在当前位置（原地抱死）。
        """
        if not self._estop_event.is_set():
            return
        self._drain_queue()   # GUI 侧已清一次，此处再清防竞态
        try:
            # ① 立即终止运动（厂商急停指令）
            if not dr.estop(0):
                raise RuntimeError('vendor estop command returned False')
            # ② 读取停止后的实际位置（~60ms；缺帧关节逐个用最近轮询快照补齐）
            frames: Dict[int, Tuple[float, float, float]] = {}
            try:
                frames = self._poll_states()
            except Exception:
                pass
            if self._last_state is not None:
                fallback = list(self._last_state.joints)
                if self._last_state.gripper is not None:
                    fallback.append(self._last_state.gripper)
                for j in fallback:
                    if j.valid and j.joint_id not in frames:
                        frames[j.joint_id] = (j.angle_deg, 0.0, 0.0)
            # ③ 重新闭环并锁定在当前位置（防止急停卸载后重力坠落；含夹爪）
            lockable = (list(self._joint_ids)
                        + ([self._gripper_id] if self._gripper_id else []))
            hold_ids = [jid for jid in lockable if jid in frames]
            relocked, states = self._request_and_verify_joint_mode(2, frames)
            if relocked and hold_ids:
                self.joints_mode_changed.emit(2)
                self.log_message.emit('warn', tr(
                    '!!! E-stop: motion halted; joints re-locked and verified '
                    'at current position (%s) !!!')
                    % {j: round(frames[j][0], 1) for j in hold_ids})
            elif not relocked:
                self._closed_loop_verified = False
                self.log_message.emit('error', tr(
                    'E-stop halted motion but closed-loop re-lock verification '
                    'FAILED: %s. Support the arm and check the drive!') % states)
            else:
                self.log_message.emit('error', tr(
                    'E-stop: motion halted but joint positions unknown — '
                    'closed loop was restored, but verify the arm is holding!'))
        except Exception as e:
            self.log_message.emit('error', tr('Error sending E-stop: %s') % e)
        finally:
            self._estop_event.clear()
        # 兜底关门（estop() 已在 GUI 侧关过；此处覆盖直接置位事件的路径）
        if self._motion_enabled:
            self._motion_enabled = False
            self.motion_enabled_changed.emit(False)
            self.log_message.emit('warn', tr('Motion enable auto-disabled after E-stop; re-enable to move again'))

    def _process_commands(self) -> None:
        """处理完队列中的全部命令后返回；每条命令之间优先检查急停/停止。"""
        while not self._stop_event.is_set():
            self._service_estop()
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            self._execute_command(cmd)

    def _execute_command(self, cmd: tuple) -> None:
        """在工作线程中执行一条命令。异常一律转日志，不允许杀死线程。"""
        kind = cmd[0]
        try:
            if kind == 'angles':
                _, id_list, angle_list, speed, acc = cmd
                # 执行时刻二次确认使能（入队后可能已被关闭；关使能虽会清队列，
                # 但存在取出命令与关使能的竞态窗口，此处兜底）
                if not self._motion_enabled:
                    self.log_message.emit('warn', tr('Motion not enabled; discarding queued angle command'))
                    return
                # Vendor set_angles(mode=1) otherwise performs blocking
                # per-axis property reads whenever its private cache flag was
                # cleared.  State polling clears that flag on every cycle.
                # Seed it from our fresh single-worker telemetry so command
                # dispatch remains write-only and deterministic.
                cached_angles = []
                state = self._last_state
                if state is not None:
                    for jid in id_list:
                        joint = state.joint(int(jid))
                        if joint is None or not joint.valid:
                            cached_angles = []
                            break
                        cached_angles.append(float(joint.angle_deg))
                if len(cached_angles) == len(angle_list):
                    dr.cur_angle_list = list(cached_angles)
                    dr.set_angles_mode_1_flag = 1
                ok = dr.set_angles(id_list=list(id_list), angle_list=list(angle_list),
                                   speed=speed, param=acc, mode=1)   # 梯形轨迹
                if ok:
                    self.log_message.emit('info', tr('Target angles sent (°): %s')
                                          % [round(a, 2) for a in angle_list])
                else:
                    self.log_message.emit('error', tr('Failed to send target angles (library returned False)'))
            elif kind == 'zero':
                if not self._motion_enabled:
                    self.log_message.emit('warn', tr('Motion not enabled; discarding queued set-zero command'))
                    return
                ok = dr.set_zero_position(0)   # 广播；会短暂卸载关节并保存配置
                if ok:
                    self.log_message.emit('warn', tr('Current position set as zero (broadcast)'))
                else:
                    self.log_message.emit('error', tr('Set zero failed (library returned False)'))
            elif kind == 'gripper':
                _, angle, speed, acc = cmd
                if not self._motion_enabled:
                    self.log_message.emit('warn', tr('Motion not enabled; queued gripper command dropped'))
                    return
                ok = dr.set_angle(id_num=self._gripper_id, angle=float(angle),
                                  speed=speed, param=acc, mode=1)
                if ok:
                    self.log_message.emit('info', tr('Gripper target sent: %.2f° (speed %.1f r/min)')
                                          % (angle, speed))
                else:
                    self.log_message.emit('error', tr('Gripper command failed (library returned False)'))
            elif kind == 'gripper_adaptive':
                _, angle, speed, torque = cmd
                if not self._motion_enabled:
                    self.log_message.emit(
                        'warn', tr('Motion not enabled; queued gripper command dropped'))
                    return
                ok = dr.set_angle_adaptive(
                    id_num=self._gripper_id, angle=float(angle),
                    speed=float(speed), torque=float(torque))
                if ok:
                    self.log_message.emit(
                        'info', 'Torque-limited gripper target sent: %.2f deg '
                        '(speed %.1f r/min, torque %.2f Nm)'
                        % (angle, speed, torque))
                else:
                    self.log_message.emit(
                        'error', 'Torque-limited gripper command failed')
            elif kind == 'gripper_zero':
                if not self._motion_enabled:
                    self.log_message.emit('warn', tr('Motion not enabled; queued gripper zero dropped'))
                    return
                ok = dr.set_zero_position(self._gripper_id)   # 仅夹爪电机，非广播
                if ok:
                    self.log_message.emit('warn', tr('Gripper closed-zero set: current position is now 0° (persisted)'))
                else:
                    self.log_message.emit('error', tr('Gripper zero failed (library returned False)'))
            elif kind == 'mode':
                _, mode = cmd
                hold_frames: Dict[int, Tuple[float, float, float]] = {}
                if mode == 2:
                    try:
                        hold_frames = self._poll_states()
                    except Exception:
                        pass
                    if self._last_state is not None:
                        fallback = list(self._last_state.joints)
                        if self._last_state.gripper is not None:
                            fallback.append(self._last_state.gripper)
                        for j in fallback:
                            if j.valid and j.joint_id not in hold_frames:
                                hold_frames[j.joint_id] = (
                                    j.angle_deg, 0.0, 0.0)
                ok, states = self._request_and_verify_joint_mode(
                    mode, hold_frames)
                if ok:
                    self.joints_mode_changed.emit(mode)
                    if mode == 1:
                        self.log_message.emit(
                            'warn', tr('Joints released (standby mode): holding torque off, '
                                       'can be moved by hand; click "Lock Joints" to '
                                       'restore closed-loop and resume control'))
                    else:
                        self.log_message.emit(
                            'info', tr('Joints locked and verified in closed loop: %s')
                            % states)
                        # 锁定即恢复可控（连接即控模式下，锁定是急停/释放后的恢复入口）
                        if not self._motion_enabled:
                            self._motion_enabled = True
                            self.motion_enabled_changed.emit(True)
                else:
                    self._closed_loop_verified = False
                    if self._motion_enabled:
                        self._motion_enabled = False
                        self.motion_enabled_changed.emit(False)
                    self.log_message.emit(
                        'error', tr('Joint mode verification failed: %s') % states)
            else:
                self.log_message.emit('warn', tr('Unknown command type: %s') % (kind,))
        except Exception as e:   # 含 TimeoutError / 串口异常
            self.log_message.emit('error', tr('Error executing command %s: %s') % (kind, e))

    def _read_joint_states(self) -> Dict[int, Optional[int]]:
        """逐轴读取现场固件 current_state；读取失败保留为 None。"""
        states: Dict[int, Optional[int]] = {}
        for jid in self._joint_ids:
            try:
                value = dr.read_property(
                    id_num=jid, property='dr.current_state')
                if (isinstance(value, (int, float))
                        and not isinstance(value, bool)
                        and math.isfinite(float(value))):
                    states[jid] = int(value)
                else:
                    states[jid] = None
            except Exception:
                states[jid] = None
        return states

    def _read_joint_errors(self) -> Dict[int, Dict[str, Optional[int]]]:
        """逐轴读取关键错误寄存器；读不到也视为预检失败。"""
        properties = (
            'dr.error',
            'dr.controller.error',
            'dr.motor.error',
            'dr.encoder.error',
        )
        result: Dict[int, Dict[str, Optional[int]]] = {}
        for jid in self._joint_ids:
            per_joint: Dict[str, Optional[int]] = {}
            for prop in properties:
                try:
                    value = dr.read_property(id_num=jid, property=prop)
                    if (isinstance(value, (int, float))
                            and not isinstance(value, bool)
                            and math.isfinite(float(value))):
                        per_joint[prop] = int(value)
                    else:
                        per_joint[prop] = None
                except Exception:
                    per_joint[prop] = None
            result[jid] = per_joint
        return result

    def _request_and_verify_joint_mode(
            self, mode: int,
            hold_frames: Optional[Dict[int, Tuple[float, float, float]]] = None
            ) -> 'tuple[bool, Dict[int, Optional[int]]]':
        """写入真实 requested_state，并逐轴读回；闭环时先锁住当前角度。"""
        requested = (self._IDLE_STATE if mode == 1
                     else self._CLOSED_LOOP_STATE)
        sent = dr.write_property(
            id_num=0, property='dr.requested_state', value=requested)
        hold_ok = True
        if sent and mode == 2 and hold_frames:
            lockable = (list(self._joint_ids)
                        + ([self._gripper_id] if self._gripper_id else []))
            hold_ids = [jid for jid in lockable if jid in hold_frames]
            if hold_ids:
                hold_ok = bool(dr.set_angles(
                    id_list=hold_ids,
                    angle_list=[float(hold_frames[jid][0])
                                for jid in hold_ids],
                    speed=8.0, param=40.0, mode=1))
        time.sleep(self._MODE_VERIFY_SETTLE_S)
        states = self._read_joint_states()
        verified = (bool(sent) and hold_ok
                    and all(states.get(jid) == requested
                            for jid in self._joint_ids))
        self._closed_loop_verified = bool(verified and mode == 2)
        return verified, states

    # ================= 内部辅助 =================
    def _drain_queue(self) -> None:
        """清空命令队列（queue.Queue 线程安全，GUI/工作线程均可调用）。"""
        try:
            while True:
                self._cmd_queue.get_nowait()
        except queue.Empty:
            pass

    def _log_throttled(self, level: str, key: str, text: str) -> None:
        """同类日志限流（仅工作线程调用），避免高频失败刷屏。"""
        now = time.monotonic()
        if now - self._log_ts.get(key, -1e9) >= self._LOG_THROTTLE_S:
            self._log_ts[key] = now
            self.log_message.emit(level, text)


# ---- i18n 词条注册（英文源文案 -> 改写前中文原文）----
register({
    'CAN backend already running, ignoring duplicate start()': 'CAN 后端已在运行，忽略重复 start()',
    'Connecting serial port %s @ %d ...': '正在连接串口 %s @ %d ...',
    'Worker thread did not exit within 3s; serial port left to process-exit cleanup':
        '工作线程未在 3s 内退出，串口交由进程退出回收',
    'Enable rejected: backend not connected': '使能被拒绝：后端未连接',
    'Motion enable is ON: commands will drive real joints!': '运动使能已开启：命令将驱动真实关节！',
    'Motion enable is OFF; pending command queue cleared': '运动使能已关闭，待发命令队列已清空',
    'Angle command rejected: backend not connected': '角度命令被拒绝：后端未连接',
    'Angle command rejected: motion not enabled (enable motion first)': '角度命令被拒绝：运动未使能（请先开启使能）',
    'Angle command rejected: empty targets': '角度命令被拒绝：目标为空',
    'Angle command rejected: unknown joint ID %s': '角度命令被拒绝：未知关节 ID %s',
    'Angle command rejected: %s': '角度命令被拒绝：%s',
    'Target angles queued: %s (speed=%.1f r/min, accel=%.1f)': '目标角度已入队: %s (speed=%.1f r/min, accel=%.1f)',
    'Motion enable auto-disabled after E-stop; re-enable to move again': '急停后运动使能已自动关闭，需重新使能才能运动',
    '!!! E-stop request submitted (command queue cleared) !!!': '!!! 急停请求已提交（命令队列已清空）!!!',
    'E-stop request: backend not connected, cannot send to bus!': '急停请求：后端未连接，无法发送到总线！',
    'Set-zero rejected: backend not connected': '设零点被拒绝：后端未连接',
    'Set-zero rejected: motion not enabled (this operation briefly unloads joints)':
        '设零点被拒绝：运动未使能（该操作会短暂卸载关节）',
    'Set-zero command queued (broadcast; joints will unload briefly)': '设零点命令已入队（广播，会短暂卸载关节）',
    'Poll rate set to %.1f Hz': '轮询频率已设为 %.1f Hz',
    'Unknown joint mode %s (1=standby/release, 2=closed-loop/lock)': '未知关节模式 %s（1=待机/释放，2=闭环/锁定）',
    'Joint mode switch rejected: backend not connected': '切换关节模式被拒绝：后端未连接',
    'Releasing joints: motion enable auto-disabled': '释放关节：运动使能已自动关闭',
    'Disconnected': '已断开',
    'Failed to open serial port %s: %s': '串口 %s 打开失败: %s',
    'Failed to clear serial receive buffer: %s': '清空串口接收缓冲失败: %s',
    'Bus scan found joint IDs: %s': '总线扫描到关节 ID: %s',
    'Bus scan got no joint response (non-fatal; check joint power/CAN wiring, polling continues)':
        '总线扫描无关节响应（不致命，请检查关节供电/CAN 接线，继续轮询）',
    'Bus scan error (non-fatal): %s': '总线扫描异常（不致命）: %s',
    'Connected %s @ %d': '已连接 %s @ %d',
    'Error reading joint states: %s': '读取关节状态异常: %s',
    'Error reading voltage/current: %s': '读取电压/电流异常: %s',
    'No joint responded this cycle (%d/%d in a row)': '本轮无任何关节响应（连续 %d/%d 轮）',
    'All reads timed out/failed for %d consecutive cycles; connection lost':
        '连续 %d 轮读取全部超时/失败，判定连接丢失',
    '!!! E-stop (broadcast) sent !!!': '!!! 急停(广播)已发出 !!!',
    'Gripper not configured (gripper.motor_id=0)': '夹爪未配置（gripper.motor_id=0）',
    'Gripper command rejected: backend not connected': '夹爪命令被拒绝：后端未连接',
    'Gripper command rejected: motion not enabled': '夹爪命令被拒绝：运动未使能',
    'Gripper zero rejected: backend not connected': '夹爪置零被拒绝：后端未连接',
    'Gripper zero rejected: motion not enabled': '夹爪置零被拒绝：运动未使能',
    'Motion not enabled; queued gripper command dropped': '运动未使能，丢弃已入队的夹爪命令',
    'Motion not enabled; queued gripper zero dropped': '运动未使能，丢弃已入队的夹爪置零',
    'Gripper target sent: %.2f° (speed %.1f r/min)': '夹爪目标已下发: %.2f°（速度 %.1f r/min）',
    'Gripper command failed (library returned False)': '夹爪命令失败（库返回 False）',
    'Gripper closed-zero set: current position is now 0° (persisted)':
        '夹爪闭合零点已设置：当前位置即 0°（掉电保存）',
    'Gripper zero failed (library returned False)': '夹爪置零失败（库返回 False）',
    '!!! E-stop: motion halted, joints locked at current position (%s) !!!':
        '!!! 急停：运动已终止，关节已锁定在当前位置 (%s) !!!',
    'E-stop: motion halted but joint positions unknown — joints re-locked in '
    'closed loop, verify the arm is holding!':
        '急停：运动已终止但关节位置未知——已重新闭环锁定，请确认机械臂保持住了！',
    'E-stop send failed (library returned False); check serial port/bus!': '急停发送失败（库返回 False），请检查串口/总线！',
    'Error sending E-stop: %s': '急停发送异常: %s',
    'Motion not enabled; discarding queued angle command': '运动未使能，丢弃已入队的角度命令',
    'Target angles sent (°): %s': '已下发目标角度(°): %s',
    'Failed to send target angles (library returned False)': '下发目标角度失败（库返回 False）',
    'Motion not enabled; discarding queued set-zero command': '运动未使能，丢弃已入队的设零点命令',
    'Current position set as zero (broadcast)': '已将当前位置设为零点(广播)',
    'Set zero failed (library returned False)': '设零点失败（库返回 False）',
    'Joints released (standby mode): holding torque off, can be moved by hand; '
    'click "Lock Joints" to restore closed-loop and resume control':
        '关节已释放(待机模式)：失去保持力矩，可手动摆位；点「锁定关节」恢复闭环后即可继续控制',
    'Joints locked (closed-loop mode): holding current position': '关节已锁定(闭环模式)：保持当前位置',
    'Failed to switch joint mode (library returned False)': '切换关节模式失败（库返回 False）',
    'Unknown command type: %s': '未知命令类型: %s',
    'Error executing command %s: %s': '执行命令 %s 异常: %s',
    'Enable rejected: J1-J4 closed-loop state was not verified':
        '使能被拒绝：J1~J4 的闭环状态尚未通过核验',
    'Joint mode switch rejected: E-stop is still being processed':
        '切换关节模式被拒绝：急停仍在处理中，请稍后再点锁定',
    'J1-J4 closed-loop verification failed: %s':
        'J1~J4 闭环状态核验失败：%s',
    'Joints locked and verified in closed loop: %s':
        '关节已锁定并逐轴确认闭环：%s',
    'Joint mode verification failed: %s': '关节模式读回核验失败：%s',
    '!!! E-stop: motion halted; joints re-locked and verified at current '
    'position (%s) !!!':
        '!!! 急停：运动已终止，关节已在当前位置重新锁定并核验 (%s) !!!',
    'E-stop halted motion but closed-loop re-lock verification FAILED: %s. '
    'Support the arm and check the drive!':
        '急停已终止运动，但重新闭环核验失败：%s。请托住机械臂并检查驱动器！',
    'E-stop: motion halted but joint positions unknown — closed loop was '
    'restored, but verify the arm is holding!':
        '急停：运动已终止但关节位置未知——已恢复闭环，请确认机械臂保持住了！',
})
