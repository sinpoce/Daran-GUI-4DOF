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
* set_zero_here 只接受明确的机械臂关节 ID，绝不使用 ID 0 广播，避免
  同一总线上的夹爪或其他电机被意外改写零点。
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
    # 状态帧突发读取超时：J1-J4 正常 <10ms 到齐；主关节缺帧时最多等 0.25s。
    _STATE_READ_TIMEOUT_S = 0.25
    # 夹爪是可选轴。主关节帧到齐后只给它一个很短的尾帧宽限，避免配置了
    # ID6 但夹爪离线时每轮都等满 0.25s、把 20Hz 示教采样拖到约 4Hz。
    _OPTIONAL_STATE_READ_TIMEOUT_S = 0.012
    # 单关节缺帧去抖：连续缺帧超过该轮数才标离线（偶发丢 1 帧不闪烁）
    _OFFLINE_AFTER_MISSES = 2
    # 电压/电流补充读取周期（秒），约 1Hz
    _VOL_CUR_PERIOD_S = 1.0
    # 同类告警日志的最小间隔（秒），避免 20Hz 失败刷屏
    _LOG_THROTTLE_S = 5.0
    # 251109 的公开 set_mode 参数与固件 current_state 不是同一枚举：
    # API 仍用 1=待机、2=闭环；真机进入闭环后 current_state 回读为 8。
    # 两组常量必须始终分开，UI 的 joints_mode 也只使用公开的 1/2。
    _STANDBY_MODE = 1
    _CLOSED_LOOP_MODE = 2
    _IDLE_STATE = 1
    _CLOSED_LOOP_STATE = 8
    _MODE_VERIFY_SETTLE_S = 0.15
    # Persistent zero writes are only allowed from a recent stationary sample.
    # Three poll periods cover normal scheduling jitter; one second is the
    # conservative floor at the normal 20 Hz rate.
    _ZERO_STATE_MIN_MAX_AGE_S = 1.0
    _ZERO_STATIONARY_SPEED_RPM = 0.5
    # 锁定前一次广播状态请求偶尔会漏掉单轴帧。在同一次安全采集中
    # 合并最多三轮新鲜响应，全部配置轴到齐即提前停止。
    _HOLD_POSE_READ_ATTEMPTS = 3

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
        # 流式轨迹会话号：Stop Playback 递增它，工作线程据此丢弃旧会话残点。
        # 整数读写在 CPython/GIL 下原子，且会话创建/取消都来自 GUI 线程。
        self._trajectory_generation = 0
        self._trajectory_point_lock = threading.Lock()
        # Non-None only while one teach-playback session owns the motion path.
        # Manual commands check this both before enqueue and again in the
        # worker, so an older queued manual move cannot interleave with mode=0
        # streaming after begin_trajectory().
        self._active_trajectory_token: Optional[int] = None
        self._pending_trajectory_point: Optional[tuple] = None
        self._executing_trajectory_token: Optional[int] = None
        self._last_trajectory_send_token: Optional[int] = None
        self._last_trajectory_send_ts = 0.0
        # 设零包含厂商 save_config()（每轴约阻塞 2 秒）。用独立 latch
        # 覆盖“已入队”和“工作线程执行中”两个阶段，阻止重复提交；所有
        # 实际串口调用仍只发生在唯一工作线程。
        self._zero_setting_lock = threading.Lock()
        self._zero_setting_active = False
        self._zero_setting_ids: Tuple[int, ...] = ()
        # shutdown() invalidates the public token before the worker exits.  A
        # separate latch preserves the obligation to lock/hold before closing
        # the serial port even though that generation is no longer valid.
        self._shutdown_hold_requested = False

        # ---- 状态缓存 ----
        # bool/引用赋值在 CPython(GIL) 下原子，GUI 线程读、工作线程写，无需额外锁
        self._connected = False
        self._last_state: Optional[ArmState] = None
        self._closed_loop_verified = False
        self._joints_mode_state = 0
        # Conservative physical-motion latch.  A vendor call may have started
        # a trapezoid even after the GUI gate is turned off; keep this true
        # until a verified current-position hold succeeds.
        self._motion_may_be_active = False
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

        id_error = ''
        if not self._joint_ids:
            id_error = tr('At least one arm joint ID is required')
        elif len(set(self._joint_ids)) != len(self._joint_ids):
            id_error = tr('Arm joint IDs must be unique: %s') % self._joint_ids
        elif any(jid < 1 or jid > 63 for jid in self._joint_ids):
            id_error = tr('Arm joint IDs must be in [1, 63]: %s') % self._joint_ids
        elif self._gripper_id and not (1 <= self._gripper_id <= 63):
            id_error = tr('Gripper ID must be 0 (disabled) or in [1, 63]')
        elif self._gripper_id in self._joint_ids:
            id_error = tr('Gripper ID %d duplicates an arm joint ID') % self._gripper_id
        if id_error:
            self.log_message.emit('error', id_error)
            self.connection_changed.emit(False, id_error)
            return

        # 复位同步原语，允许 shutdown 后再次 start
        self._stop_event.clear()
        self._estop_event.clear()
        self._wake_event.clear()
        self._drain_queue()
        with self._trajectory_point_lock:
            self._trajectory_generation += 1
            self._active_trajectory_token = None
            self._pending_trajectory_point = None
            self._executing_trajectory_token = None
            self._last_trajectory_send_token = None
            self._last_trajectory_send_ts = 0.0
        self._shutdown_hold_requested = False
        self._log_ts.clear()
        # 兜底：上次会话的使能状态不得带入新连接（运动默认禁止）
        self._closed_loop_verified = False
        self._joints_mode_state = 0
        self._motion_may_be_active = False
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
        with self._trajectory_point_lock:
            self._shutdown_hold_requested = bool(
                self._shutdown_hold_requested
                or self._active_trajectory_token is not None
                or self._executing_trajectory_token is not None
                or self._motion_enabled
                or self._motion_may_be_active
                or self._joints_mode_state == self._STANDBY_MODE)
            self._trajectory_generation += 1
            self._active_trajectory_token = None
            self._pending_trajectory_point = None
            self._last_trajectory_send_token = None
            self._last_trajectory_send_ts = 0.0
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
            with self._trajectory_point_lock:
                if self._active_trajectory_token is not None:
                    self._trajectory_generation += 1
                self._active_trajectory_token = None
                self._pending_trajectory_point = None
                self._last_trajectory_send_token = None
                self._last_trajectory_send_ts = 0.0
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
        if self._manual_motion_blocked('Angle command'):
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
        if not self._enqueue_manual_command(
                ('angles', id_list, angle_list, speed, acc), 'Angle command'):
            return
        self.log_message.emit('info', tr('Target angles queued: %s (speed=%.1f r/min, accel=%.1f)')
                              % ({j: round(a, 2) for j, a in zip(id_list, angle_list)},
                                 speed, acc))

    # ================= 示教流式轨迹（GUI 线程调用） =================
    def begin_trajectory(self) -> int:
        """Create a new teach-playback generation and invalidate older points."""
        # Lock order is always zero-setting -> trajectory when both are needed.
        # This atomically prevents zero-setting and a new teach owner from being
        # established at the same time by different callers.
        with self._zero_setting_lock:
            if self._zero_setting_active:
                self.log_message.emit(
                    'warn', tr('%s rejected: arm zero-setting operation is active')
                    % 'Teach trajectory start')
                return 0
            with self._trajectory_point_lock:
                self._trajectory_generation += 1
                self._active_trajectory_token = self._trajectory_generation
                self._pending_trajectory_point = None
                self._last_trajectory_send_token = None
                self._last_trajectory_send_ts = 0.0
        return self._trajectory_generation

    def command_trajectory_start(
            self, token: int, angles_deg: Dict[int, float],
            speed_rpm: float, accel: float,
            gripper_deg: Optional[float] = None) -> None:
        """Queue the reference-library mode=1 move to a recording's first point."""
        if not self._trajectory_token_is_active(token):
            return
        prepared = self._prepare_trajectory_targets(angles_deg, gripper_deg)
        if prepared is None:
            return
        id_list, angle_list = prepared
        speed, acc = self.clamp_motion_params(speed_rpm, accel)
        if not self._enqueue_manual_command(
                ('trajectory_start', int(token), id_list, angle_list, speed, acc),
                'Teach trajectory start'):
            return

    def command_trajectory_point(
            self, token: int, angles_deg: Dict[int, float],
            bandwidth_hz: float,
            gripper_deg: Optional[float] = None) -> None:
        """Queue one dense point using set_angles(mode=0) trajectory tracking.

        Per the 251109 vendor example, ``param`` is half the point-send
        frequency in this mode.  It is an input-filter bandwidth, not the
        trapezoidal acceleration shown elsewhere in the GUI.
        """
        if not self._trajectory_token_is_active(token):
            return
        if (not math.isfinite(float(bandwidth_hz))
                or float(bandwidth_hz) <= 0.0):
            self.log_message.emit(
                'error', tr('Trajectory point rejected: bandwidth must be positive'))
            return
        prepared = self._prepare_trajectory_targets(angles_deg, gripper_deg)
        if prepared is None:
            return
        id_list, angle_list = prepared
        # Vendor documentation requires bandwidth < 300.
        bandwidth = max(0.1, min(float(bandwidth_hz), 299.0))
        point = ('trajectory_point', int(token), id_list, angle_list, bandwidth)
        with self._trajectory_point_lock:
            if (int(token) != self._trajectory_generation
                    or int(token) != self._active_trajectory_token):
                return
            # A state/property read can occasionally delay the worker.  Keep
            # only the newest time sample instead of replaying stale points as
            # a burst with the wrong effective frequency.
            self._pending_trajectory_point = point
        self._wake_event.set()

    def cancel_trajectory(self, token: int, hold_position: bool = True) -> None:
        """Invalidate stale points; optionally replace the live target by a hold."""
        with self._trajectory_point_lock:
            if (int(token) != self._trajectory_generation
                    or int(token) != self._active_trajectory_token):
                return
            self._trajectory_generation += 1
            generation = self._trajectory_generation
            self._active_trajectory_token = None
            self._pending_trajectory_point = None
            self._last_trajectory_send_token = None
            self._last_trajectory_send_ts = 0.0
        if hold_position and self.is_connected() and self._motion_enabled:
            self._cmd_queue.put(('trajectory_hold', generation))
            self._wake_event.set()

    def is_trajectory_idle(self, token: int) -> bool:
        with self._trajectory_point_lock:
            pending = self._pending_trajectory_point
            pending_for_token = (pending is not None and pending[1] == int(token))
            executing_for_token = (
                self._executing_trajectory_token == int(token))
            return not pending_for_token and not executing_for_token

    def _prepare_trajectory_targets(
            self, angles_deg: Dict[int, float],
            gripper_deg: Optional[float]
            ) -> 'Optional[Tuple[List[int], List[float]]]':
        """Validate and order a J1-J4(+optional gripper) teach sample."""
        if self.is_setting_zero():
            self.log_message.emit(
                'warn', tr('%s rejected: arm zero-setting operation is active')
                % 'Teach trajectory command')
            return None
        if not self.is_connected():
            self.log_message.emit(
                'warn', tr('Trajectory command rejected: backend not connected'))
            return None
        if not self._motion_enabled:
            self.log_message.emit(
                'warn', tr('Trajectory command rejected: motion not enabled'))
            return None
        if not angles_deg:
            self.log_message.emit(
                'warn', tr('Trajectory command rejected: empty targets'))
            return None
        unknown = [int(jid) for jid in angles_deg
                   if int(jid) not in self._joint_ids]
        if unknown:
            self.log_message.emit(
                'error', tr('Trajectory command rejected: unknown joint ID %s')
                % unknown)
            return None
        normalized = {int(jid): float(value)
                      for jid, value in angles_deg.items()}
        if not all(math.isfinite(value) for value in normalized.values()):
            self.log_message.emit(
                'error', tr('Trajectory command rejected: non-finite target'))
            return None
        ok, reason = self.validate_targets(
            normalized, self._last_state, check_step=False)
        if not ok:
            self.log_message.emit(
                'error', tr('Trajectory command rejected: %s') % reason)
            return None
        id_list = [jid for jid in self._joint_ids if jid in normalized]
        angle_list = [normalized[jid] for jid in id_list]
        if gripper_deg is not None:
            if not self._gripper_id:
                self.log_message.emit(
                    'warn', tr('Trajectory gripper sample ignored: gripper not configured'))
            elif not math.isfinite(float(gripper_deg)):
                self.log_message.emit(
                    'error', tr('Trajectory command rejected: non-finite target'))
                return None
            else:
                id_list.append(self._gripper_id)
                angle_list.append(float(gripper_deg))
        return id_list, angle_list

    def estop(self) -> None:
        """急停（广播）。任何状态下可调用（包括未使能），优先于队列中其他命令。"""
        with self._trajectory_point_lock:
            self._trajectory_generation += 1
            self._active_trajectory_token = None
            self._pending_trajectory_point = None
            self._last_trajectory_send_token = None
            self._last_trajectory_send_ts = 0.0
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

    def set_zero_here(self, joint_ids: Optional[List[int]] = None) -> None:
        """把指定机械臂关节的当前位置设为持久零点（绝不广播）。"""
        if joint_ids is None:
            self.log_message.emit(
                'warn', tr('Set-zero rejected: explicit arm joint IDs are required'))
            return
        try:
            raw_ids = list(joint_ids)
            if any(not isinstance(jid, int) or isinstance(jid, bool)
                   for jid in raw_ids):
                raise ValueError('joint IDs must be integers')
            normalized = list(raw_ids)
        except (TypeError, ValueError):
            self.log_message.emit(
                'warn', tr('Set-zero rejected: invalid joint ID list %s') % joint_ids)
            return
        if not normalized:
            self.log_message.emit(
                'warn', tr('Set-zero rejected: no arm joints selected'))
            return
        if len(set(normalized)) != len(normalized):
            self.log_message.emit(
                'warn', tr('Set-zero rejected: duplicate joint IDs %s') % normalized)
            return
        unknown = [jid for jid in normalized if jid not in self._joint_ids]
        if unknown:
            self.log_message.emit(
                'warn', tr('Set-zero rejected: IDs %s are not configured arm joints')
                % unknown)
            return
        if not self.is_connected():
            self.log_message.emit('warn', tr('Set-zero rejected: backend not connected'))
            return
        if not self._motion_enabled:
            self.log_message.emit('warn', tr('Set-zero rejected: motion not enabled (this operation briefly unloads joints)'))
            return
        if self._manual_motion_blocked(
                'Set-zero command', allow_zero_setting=True):
            return
        if self.is_setting_zero():
            self.log_message.emit(
                'warn', tr('Set-zero rejected: another zero-setting operation is active'))
            return
        state_error = self._zero_target_state_error(normalized)
        if state_error:
            self.log_message.emit('warn', state_error)
            return
        with self._zero_setting_lock:
            if self._zero_setting_active:
                self.log_message.emit(
                    'warn', tr('Set-zero rejected: another zero-setting operation is active'))
                return
            with self._trajectory_point_lock:
                active_trajectory = self._active_trajectory_token
            if active_trajectory is not None:
                self.log_message.emit(
                    'warn', tr('%s rejected: teach trajectory %d is active')
                    % ('Set-zero command', active_trajectory))
                return
            if not self._cmd_queue.empty():
                self.log_message.emit(
                    'warn', tr('Set-zero rejected: command queue is not idle'))
                return
            self._zero_setting_active = True
            self._zero_setting_ids = tuple(normalized)
            ids = tuple(normalized)
            self._cmd_queue.put(('zero', ids))
        self._wake_event.set()
        self.zero_setting_changed.emit(True, list(ids))
        self.log_message.emit(
            'warn', tr('Set-zero command queued for arm joint IDs %s '
                       '(persistent; joints will unload briefly)') % list(ids))

    def is_setting_zero(self) -> bool:
        """Return True from enqueue until the worker publishes a result."""
        with self._zero_setting_lock:
            return bool(self._zero_setting_active)

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
        if self._manual_motion_blocked('Gripper command'):
            return
        speed = self._gripper_speed if speed_rpm is None else float(speed_rpm)
        acc = self._gripper_accel if accel is None else float(accel)
        if not self._enqueue_manual_command(
                ('gripper', float(angle_deg), speed, acc), 'Gripper command'):
            return

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
        if self._manual_motion_blocked('Adaptive gripper command'):
            return
        if speed_rpm <= 0.0 or torque_nm <= 0.0:
            self.log_message.emit(
                'error', 'Adaptive gripper speed and torque must be positive')
            return
        if not self._enqueue_manual_command(
                ('gripper_adaptive', float(angle_deg), float(speed_rpm),
                 float(torque_nm)), 'Adaptive gripper command'):
            return

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
        if self._manual_motion_blocked('Gripper zero'):
            return
        if not self._enqueue_manual_command(
                ('gripper_zero',), 'Gripper zero'):
            return

    def set_joints_mode(self, mode: int) -> None:
        """广播切换关节模式：1=待机(卸载/可手推)，2=闭环(锁定保持)。

        释放(1)后关节失去保持力矩（可手动摆位），同时自动关闭运动使能门；
        厂商约定运动控制必须在闭环模式下执行，故释放后需先「锁定」再使能运动。
        """
        if mode not in (self._STANDBY_MODE, self._CLOSED_LOOP_MODE):
            self.log_message.emit('error', tr('Unknown joint mode %s (1=standby/release, 2=closed-loop/lock)') % mode)
            return
        if not self.is_connected():
            self.log_message.emit('warn', tr('Joint mode switch rejected: backend not connected'))
            return
        if self._estop_event.is_set():
            self.log_message.emit(
                'warn', tr('Joint mode switch rejected: E-stop is still being processed'))
            return
        if (mode == self._CLOSED_LOOP_MODE
                and self._manual_motion_blocked(
                    'Joint lock', allow_zero_setting=True)):
            return
        if mode == self._STANDBY_MODE:
            # 释放 = 脱离软件控制：关使能门并清空待发命令（同急停语义）
            # Latch before enqueue: shutdown can race the worker between the
            # physical mode-1 write and its state bookkeeping.  Keeping this
            # set for the connection guarantees a final direct re-lock.
            self._shutdown_hold_requested = True
            with self._trajectory_point_lock:
                self._trajectory_generation += 1
                self._active_trajectory_token = None
                self._pending_trajectory_point = None
                self._last_trajectory_send_token = None
                self._last_trajectory_send_ts = 0.0
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
            except (Exception, SystemExit) as e:
                self.log_message.emit('warn', tr('Failed to clear serial receive buffer: %s') % e)

            # ---- 3. 总线扫描（失败不致命，仅日志提示）----
            try:
                ids = dr.get_ids()
                if ids:
                    self.log_message.emit('info', tr('Bus scan found joint IDs: %s') % ids)
                else:
                    self.log_message.emit('warn', tr('Bus scan got no joint response (non-fatal; check joint power/CAN wiring, polling continues)'))
            except (Exception, SystemExit) as e:
                self.log_message.emit('warn', tr('Bus scan error (non-fatal): %s') % e)

            # ---- 3.5 逐轴核验固件状态（idle=1, closed-loop=8）----
            mode_ids = self._configured_motor_ids()
            states = self._read_joint_states(mode_ids)
            self._drive_errors = self._read_joint_errors()
            all_closed = (len(states) == len(mode_ids)
                          and all(states.get(jid) == self._CLOSED_LOOP_STATE
                                  for jid in mode_ids))
            errors_readable = all(
                all(v is not None for v in per_joint.values())
                for per_joint in self._drive_errors.values())
            errors_clear = (len(self._drive_errors) == len(self._joint_ids)
                            and errors_readable
                            and all(v == 0
                                    for per_joint in self._drive_errors.values()
                                    for v in per_joint.values()))
            all_idle = (len(states) == len(mode_ids)
                        and all(states.get(jid) == self._IDLE_STATE
                                for jid in mode_ids))
            self._closed_loop_verified = all_closed and errors_clear
            init_mode = (self._CLOSED_LOOP_MODE if all_closed
                         else (self._STANDBY_MODE if all_idle else 0))
            self._joints_mode_state = init_mode
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
        except (Exception, SystemExit) as e:
            # The upstream 251109 read_property path calls sys.exit() on an
            # empty reply.  SystemExit is not an Exception, so keep this outer
            # containment boundary as a final guarantee that vendor code can
            # never silently kill the sole CAN worker.
            disconnect_reason = tr('CAN worker vendor call failed: %s') % e
            self.log_message.emit('error', disconnect_reason)
        finally:
            # A generation invalidation performed by shutdown() must not make
            # its final stop command disappear.  If joints were released or
            # motion/streaming was active, directly acquire a fresh pose and
            # best-effort re-lock/hold before the worker closes its own port.
            should_hold = bool(
                connected_once and ser is not None
                and getattr(dr, 'uart', None) is ser
                and getattr(ser, 'is_open', True)
                and (self._shutdown_hold_requested
                     or self._joints_mode_state == self._STANDBY_MODE
                     or self._motion_enabled
                     or self._motion_may_be_active
                     or self._active_trajectory_token is not None
                     or self._executing_trajectory_token is not None))
            if should_hold:
                try:
                    held, states, _ = self._safe_hold_current_position(
                        require_fresh=False)
                    if held:
                        self.log_message.emit(
                            'info', tr('Joints safely locked and held before serial close'))
                    else:
                        self.log_message.emit(
                            'error', tr('Final joint lock/hold before serial close failed: %s')
                            % states)
                except (Exception, SystemExit) as e:
                    self.log_message.emit(
                        'error', tr('Final joint lock/hold before serial close failed: %s')
                        % e)
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
            with self._trajectory_point_lock:
                self._active_trajectory_token = None
                self._pending_trajectory_point = None
                self._executing_trajectory_token = None
                self._last_trajectory_send_token = None
                self._last_trajectory_send_ts = 0.0
            self._shutdown_hold_requested = False
            # 断连即复位使能门：同一实例再次 start() 不得带着旧使能状态
            self._closed_loop_verified = False
            self._joints_mode_state = 0
            self._motion_may_be_active = False
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
            except (Exception, SystemExit) as e:   # 含厂商 sys.exit / 串口异常
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
                except (Exception, SystemExit) as e:
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
        # Read the required arm frames with the normal timeout.  A configured
        # gripper is optional, so wait only a short grace period for its tail
        # frame; otherwise an unplugged ID6 would throttle teaching to ~4Hz.
        required_n = max(1, len(self._joint_ids))
        old_timeout = ser.timeout
        try:
            ser.timeout = self._STATE_READ_TIMEOUT_S
            raw = ser.read(16 * required_n)
            if self._gripper_id:
                ser.timeout = self._OPTIONAL_STATE_READ_TIMEOUT_S
                raw += ser.read(16)
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
            # ②/③ worker 内直接重读位置、核验闭环，再零位移保持；急停
            # 允许用最近快照补缺，尽最大努力避免失去保持力矩。
            relocked, states, frames = self._safe_hold_current_position(
                require_fresh=False)
            hold_ids = [jid for jid in self._configured_motor_ids()
                        if jid in frames]
            if relocked:
                self._joints_mode_state = self._CLOSED_LOOP_MODE
                self.joints_mode_changed.emit(self._CLOSED_LOOP_MODE)
                self.log_message.emit('warn', tr(
                    '!!! E-stop: motion halted; joints re-locked and verified '
                    'at current position (%s) !!!')
                    % {j: round(frames[j][0], 1) for j in hold_ids})
            else:
                self._joints_mode_state = 0
                self._closed_loop_verified = False
                self.log_message.emit('error', tr(
                    'E-stop halted motion but closed-loop re-lock verification '
                    'FAILED: %s. Support the arm and check the drive!') % states)
        except (Exception, SystemExit) as e:
            self.log_message.emit('error', tr('Error sending E-stop: %s') % e)
        finally:
            self._estop_event.clear()
        # 兜底关门（estop() 已在 GUI 侧关过；此处覆盖直接置位事件的路径）
        if self._motion_enabled:
            self._motion_enabled = False
            self.motion_enabled_changed.emit(False)
            self.log_message.emit('warn', tr('Motion enable auto-disabled after E-stop; re-enable to move again'))

    def _process_commands(self) -> None:
        """Process normal commands, then at most one coalesced trajectory point."""
        while not self._stop_event.is_set():
            self._service_estop()
            from_trajectory_slot = False
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                cmd = self._take_pending_trajectory_point()
                if cmd is None:
                    return
                from_trajectory_slot = True
            self._execute_command(cmd)
            if from_trajectory_slot:
                # Publish fresh telemetry between dense points.  If the GUI
                # submitted several points while I/O was delayed, the slot has
                # already retained only the newest one for the next cycle.
                return

    def _take_pending_trajectory_point(self) -> Optional[tuple]:
        with self._trajectory_point_lock:
            cmd = self._pending_trajectory_point
            self._pending_trajectory_point = None
            # Mark ownership while still holding the same lock used by
            # is_trajectory_idle().  This closes the pending->executing gap in
            # which the GUI could otherwise cancel the token before the final
            # point had actually reached the vendor API.
            if cmd is not None:
                self._executing_trajectory_token = int(cmd[1])
        return cmd

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
                if self._manual_motion_blocked('Queued angle command'):
                    return
                self._seed_vendor_angle_cache(id_list)
                self._motion_may_be_active = True
                ok = dr.set_angles(id_list=list(id_list), angle_list=list(angle_list),
                                   speed=speed, param=acc, mode=1)   # 梯形轨迹
                if ok:
                    self.log_message.emit('info', tr('Target angles sent (°): %s')
                                          % [round(a, 2) for a in angle_list])
                else:
                    self.log_message.emit('error', tr('Failed to send target angles (library returned False)'))
            elif kind == 'trajectory_start':
                _, token, id_list, angle_list, speed, acc = cmd
                if not self._trajectory_token_is_active(token):
                    return
                if self.is_setting_zero():
                    self.log_message.emit(
                        'warn', tr('%s rejected: arm zero-setting operation is active')
                        % 'Queued teach trajectory start')
                    return
                if not self._motion_enabled:
                    self.log_message.emit(
                        'warn', tr('Motion not enabled; discarding queued trajectory command'))
                    return
                self._seed_vendor_angle_cache(id_list)
                self._motion_may_be_active = True
                ok = dr.set_angles(
                    id_list=list(id_list), angle_list=list(angle_list),
                    speed=float(speed), param=float(acc), mode=1)
                if ok:
                    self.log_message.emit(
                        'info', tr('Teach trajectory moving to first point (°): %s')
                        % [round(a, 2) for a in angle_list])
                else:
                    self.log_message.emit(
                        'error', tr('Failed to move teach trajectory to first point'))
            elif kind == 'trajectory_point':
                _, token, id_list, angle_list, bandwidth = cmd
                try:
                    if not self._trajectory_token_is_active(token):
                        return
                    if self.is_setting_zero():
                        self.log_message.emit(
                            'warn', tr('%s rejected: arm zero-setting operation is active')
                            % 'Queued teach trajectory point')
                        return
                    if not self._motion_enabled:
                        self.log_message.emit(
                            'warn', tr('Motion not enabled; discarding queued trajectory command'))
                        return
                    # 251109 reference workflow: dense samples use tracking
                    # mode=0; param is send frequency / 2.
                    now = time.monotonic()
                    if (self._last_trajectory_send_token == token
                            and self._last_trajectory_send_ts > 0.0):
                        actual_dt = now - self._last_trajectory_send_ts
                        if actual_dt > 0.0:
                            bandwidth = min(
                                float(bandwidth), max(0.1, 0.5 / actual_dt))
                    self._motion_may_be_active = True
                    ok = dr.set_angles(
                        id_list=list(id_list), angle_list=list(angle_list),
                        speed=0.0, param=float(bandwidth), mode=0)
                    if ok:
                        self._last_trajectory_send_token = token
                        self._last_trajectory_send_ts = time.monotonic()
                    if not ok:
                        self.log_message.emit(
                            'error', tr('Failed to send streamed trajectory point'))
                finally:
                    with self._trajectory_point_lock:
                        if self._executing_trajectory_token == token:
                            self._executing_trajectory_token = None
            elif kind == 'trajectory_hold':
                _, token = cmd
                if (token != self._trajectory_generation
                        or self._active_trajectory_token is not None
                        or not self._motion_enabled):
                    return
                ok, states, _ = self._safe_hold_current_position(
                    require_fresh=True)
                if ok:
                    self.log_message.emit(
                        'info', tr('Teach trajectory stopped and current pose is held'))
                else:
                    self.log_message.emit(
                        'error', tr('Failed to hold pose after stopping teach trajectory: %s')
                        % states)
            elif kind == 'zero':
                _, selected_ids = cmd
                requested = list(selected_ids)
                succeeded: List[int] = []
                failed: List[int] = []
                reason = ''
                try:
                    if not self._motion_enabled:
                        reason = tr('Motion not enabled; discarding queued set-zero command')
                        self.log_message.emit('warn', reason)
                        failed.extend(requested)
                        return
                    if self._manual_motion_blocked(
                            'Queued set-zero command', allow_zero_setting=True):
                        reason = tr('Set-zero cancelled: teach trajectory is active')
                        failed.extend(requested)
                        return
                    state_error = self._zero_target_state_error(requested)
                    if state_error:
                        reason = state_error
                        self.log_message.emit('warn', state_error)
                        failed.extend(requested)
                        return
                    self._motion_may_be_active = True
                    for index, jid in enumerate(requested):
                        # save_config() in the 251109 public API sleeps for about
                        # two seconds.  Check cancellation between axes so E-stop,
                        # shutdown, or motion-disable does not start another write.
                        if (self._stop_event.is_set()
                                or self._estop_event.is_set()
                                or not self._motion_enabled):
                            reason = tr('Set-zero interrupted before all selected joints were written')
                            failed.extend(requested[index:])
                            break
                        try:
                            # An explicit non-zero ID is essential: ID 0 broadcasts
                            # to the gripper and every other motor on the CAN bus.
                            ok = bool(dr.set_zero_position(int(jid)))
                        except (Exception, SystemExit) as e:
                            ok = False
                            self.log_message.emit(
                                'error', tr('Set zero failed for arm joint ID %d: %s')
                                % (jid, e))
                        if ok:
                            succeeded.append(jid)
                            self.log_message.emit(
                                'warn', tr('Arm joint ID %d current position set '
                                           'as zero (persisted)') % jid)
                        else:
                            failed.append(jid)
                            self.log_message.emit(
                                'error', tr('Set zero failed for arm joint ID %d '
                                            '(library returned False)') % jid)
                    if not failed:
                        self.log_message.emit(
                            'warn', tr('Set zero completed for arm joint IDs %s')
                            % succeeded)
                    else:
                        self.log_message.emit(
                            'error', tr('Set zero finished with failures; '
                                        'succeeded=%s, failed=%s')
                            % (succeeded, failed))
                finally:
                    # Cover early returns and unexpected errors so the GUI busy
                    # state can never remain latched indefinitely.
                    accounted = set(succeeded) | set(failed)
                    failed.extend(jid for jid in requested if jid not in accounted)
                    self._finish_zero_setting(
                        requested, succeeded, failed, reason)
            elif kind == 'gripper':
                _, angle, speed, acc = cmd
                if not self._motion_enabled:
                    self.log_message.emit('warn', tr('Motion not enabled; queued gripper command dropped'))
                    return
                if self._manual_motion_blocked('Queued gripper command'):
                    return
                self._motion_may_be_active = True
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
                if self._manual_motion_blocked('Queued adaptive gripper command'):
                    return
                self._motion_may_be_active = True
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
                if self._manual_motion_blocked('Queued gripper zero'):
                    return
                self._motion_may_be_active = True
                ok = dr.set_zero_position(self._gripper_id)   # 仅夹爪电机，非广播
                if ok:
                    self.log_message.emit('warn', tr('Gripper closed-zero set: current position is now 0° (persisted)'))
                else:
                    self.log_message.emit('error', tr('Gripper zero failed (library returned False)'))
            elif kind == 'mode':
                _, mode = cmd
                if mode == self._CLOSED_LOOP_MODE:
                    if self._manual_motion_blocked(
                            'Queued joint lock', allow_zero_setting=True):
                        return
                    ok, states, _ = self._safe_hold_current_position(
                        require_fresh=True)
                else:
                    ok, states = self._request_and_verify_joint_mode(mode)
                if ok:
                    self._joints_mode_state = int(mode)
                    self.joints_mode_changed.emit(mode)
                    if mode == self._STANDBY_MODE:
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
                    self._joints_mode_state = 0
                    self._closed_loop_verified = False
                    # Publish the failed/unknown transition as well.  Leaving
                    # the last mode (often 1=released) cached in the GUI made a
                    # failed post-record lock look as if its asynchronous
                    # request had simply disappeared and gave retry owners no
                    # fresh state to display.
                    self.joints_mode_changed.emit(0)
                    if self._motion_enabled:
                        self._motion_enabled = False
                        self.motion_enabled_changed.emit(False)
                    self.log_message.emit(
                        'error', tr('Joint mode verification failed: %s') % states)
            else:
                self.log_message.emit('warn', tr('Unknown command type: %s') % (kind,))
        except (Exception, SystemExit) as e:   # 厂商库会在无回复时 sys.exit()
            if kind == 'trajectory_point' and len(cmd) > 1:
                with self._trajectory_point_lock:
                    if self._executing_trajectory_token == cmd[1]:
                        self._executing_trajectory_token = None
            self.log_message.emit('error', tr('Error executing command %s: %s') % (kind, e))

    def _configured_motor_ids(self) -> List[int]:
        """Return every configured motor affected by broadcast set_mode()."""
        ids = list(self._joint_ids)
        if self._gripper_id and self._gripper_id not in ids:
            ids.append(self._gripper_id)
        return ids

    def _trajectory_token_is_active(self, token: int) -> bool:
        """Atomically validate both the generation and active ownership."""
        try:
            normalized = int(token)
        except (TypeError, ValueError):
            return False
        with self._trajectory_point_lock:
            return (normalized == self._trajectory_generation
                    and normalized == self._active_trajectory_token)

    def _manual_motion_blocked(
            self, action: str, allow_zero_setting: bool = False) -> bool:
        """Reject vendor motion while teach streaming or zero-writing owns it.

        E-stop and joint mode/hold safety paths pass ``allow_zero_setting`` so
        they remain available to make the arm safe.  The zero command itself
        also passes it because its own busy latch is necessarily already set.
        """
        if not allow_zero_setting and self.is_setting_zero():
            self.log_message.emit(
                'warn', tr('%s rejected: arm zero-setting operation is active')
                % action)
            return True
        with self._trajectory_point_lock:
            active = self._active_trajectory_token
        if active is None:
            return False
        self.log_message.emit(
            'warn', tr('%s rejected: teach trajectory %d is active')
            % (action, active))
        return True

    def _enqueue_manual_command(self, cmd: tuple, action: str) -> bool:
        """Atomically exclude manual enqueues from a persistent zero write.

        The earlier public validation improves error messages, while this final
        check closes the race where another caller begins zero-setting between
        validation and queue insertion.
        """
        with self._zero_setting_lock:
            if self._zero_setting_active:
                self.log_message.emit(
                    'warn', tr('%s rejected: arm zero-setting operation is active')
                    % action)
                return False
            self._cmd_queue.put(cmd)
        self._wake_event.set()
        return True

    def _seed_vendor_angle_cache(self, id_list: List[int]) -> None:
        """Avoid the vendor mode=1 helper doing hidden blocking property reads."""
        # Never let an equally-sized command for different IDs reuse an older
        # angle list when the current snapshot is incomplete.
        dr.cur_angle_list = []
        dr.set_angles_mode_1_flag = 0
        cached_angles: List[float] = []
        state = self._last_state
        if state is not None:
            for jid in id_list:
                reading = state.joint(int(jid))
                if (reading is None and state.gripper is not None
                        and state.gripper.joint_id == int(jid)):
                    reading = state.gripper
                if reading is None or not reading.valid:
                    cached_angles = []
                    break
                cached_angles.append(float(reading.angle_deg))
        if len(cached_angles) == len(id_list):
            dr.cur_angle_list = list(cached_angles)
            dr.set_angles_mode_1_flag = 1

    def _live_target_lists(self) -> 'Tuple[List[int], List[float]]':
        """Return the latest valid arm(+gripper) pose in deterministic ID order."""
        ids: List[int] = []
        angles: List[float] = []
        state = self._last_state
        if state is None:
            return ids, angles
        for jid in self._joint_ids:
            reading = state.joint(jid)
            if reading is not None and reading.valid:
                ids.append(jid)
                angles.append(float(reading.angle_deg))
        gripper = state.gripper
        if (self._gripper_id and gripper is not None and gripper.valid
                and gripper.joint_id == self._gripper_id):
            ids.append(self._gripper_id)
            angles.append(float(gripper.angle_deg))
        return ids, angles

    def _cached_pose_frames(self) -> Dict[int, Tuple[float, float, float]]:
        """Build best-effort hold frames from the latest published snapshot."""
        out: Dict[int, Tuple[float, float, float]] = {}
        state = self._last_state
        if state is None:
            return out
        for jid in self._configured_motor_ids():
            reading = state.joint(jid)
            if (reading is None and state.gripper is not None
                    and state.gripper.joint_id == jid):
                reading = state.gripper
            if (reading is not None and reading.valid
                    and math.isfinite(float(reading.angle_deg))):
                out[jid] = (float(reading.angle_deg), 0.0, 0.0)
        return out

    def _safe_hold_current_position(
            self, require_fresh: bool = True
            ) -> 'tuple[bool, Dict[int, Optional[int]], Dict[int, Tuple[float, float, float]]]':
        """Worker-only direct lock/hold sequence, independent of queue tokens.

        Normal lock and trajectory-stop paths require a bounded fresh-pose
        acquisition containing every configured motor.  Responses from up to
        three new broadcast requests are merged so one transient missing frame
        does not reject a safe lock.  E-stop/finally may request best effort,
        in which case any still-missing axes are supplemented from the last
        published snapshot.  In both cases public set_mode(2) is verified via
        firmware current_state=8 before the zero-displacement mode=1 hold.
        """
        required_ids = self._configured_motor_ids()
        fresh: Dict[int, Tuple[float, float, float]] = {}
        attempts = max(1, int(self._HOLD_POSE_READ_ATTEMPTS))
        for _attempt in range(attempts):
            try:
                batch = self._poll_states()
                for jid in required_ids:
                    frame = batch.get(jid)
                    if frame is not None:
                        fresh[jid] = frame
            except (Exception, SystemExit) as e:
                self._log_throttled(
                    'warn', 'safe-hold-poll',
                    tr('Unable to read fresh pose for joint hold: %s') % e)
            if all(jid in fresh for jid in required_ids):
                break

        frames = dict(fresh)
        if not require_fresh:
            for jid, frame in self._cached_pose_frames().items():
                frames.setdefault(jid, frame)

        missing_fresh = [jid for jid in required_ids if jid not in fresh]
        if require_fresh and missing_fresh:
            self._closed_loop_verified = False
            states = {jid: None for jid in required_ids}
            self.log_message.emit(
                'error', tr('Joint hold rejected: fresh position missing for IDs %s')
                % missing_fresh)
            return False, states, frames

        ok, states = self._request_and_verify_joint_mode(
            self._CLOSED_LOOP_MODE, frames,
            require_fresh=require_fresh)
        return ok, states, frames

    def _read_joint_states(
            self, ids: Optional[List[int]] = None) -> Dict[int, Optional[int]]:
        """逐轴读取现场固件 current_state；读取失败保留为 None。"""
        states: Dict[int, Optional[int]] = {}
        for jid in (self._joint_ids if ids is None else ids):
            try:
                value = dr.read_property(
                    id_num=jid, property='dr.current_state')
                if (isinstance(value, (int, float))
                        and not isinstance(value, bool)
                        and math.isfinite(float(value))):
                    states[jid] = int(value)
                else:
                    states[jid] = None
            except (Exception, SystemExit):
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
                except (Exception, SystemExit):
                    per_joint[prop] = None
            result[jid] = per_joint
        return result

    def _request_and_verify_joint_mode(
            self, mode: int,
            hold_frames: Optional[Dict[int, Tuple[float, float, float]]] = None,
            require_fresh: bool = True,
            ) -> 'tuple[bool, Dict[int, Optional[int]]]':
        """Use the 251109 API and verify every broadcast-targeted motor.

        For mode 2, a complete current-position snapshot is a prerequisite in
        the normal path.  The broadcast is sent first, all arm and configured
        gripper states are then verified as closed loop, and only then is an
        exact-current-position mode=1 command sent to establish a safe hold.
        """
        if mode not in (self._STANDBY_MODE, self._CLOSED_LOOP_MODE):
            states = {jid: None for jid in self._configured_motor_ids()}
            self._closed_loop_verified = False
            return False, states
        requested = (self._IDLE_STATE if mode == self._STANDBY_MODE
                     else self._CLOSED_LOOP_STATE)
        target_ids = self._configured_motor_ids()
        positions: Dict[int, float] = {}
        if mode == self._CLOSED_LOOP_MODE:
            for jid in target_ids:
                frame = (hold_frames or {}).get(jid)
                if frame is None:
                    continue
                try:
                    angle = float(frame[0])
                except (TypeError, ValueError, IndexError):
                    continue
                if math.isfinite(angle):
                    positions[jid] = angle
            missing = [jid for jid in target_ids if jid not in positions]
            if require_fresh and missing:
                # Crucially do not switch mode if we cannot immediately hold
                # every motor affected by the broadcast.
                self._closed_loop_verified = False
                self.log_message.emit(
                    'error', tr('Joint lock rejected: current position missing for IDs %s')
                    % missing)
                return False, {jid: None for jid in target_ids}

        # Do not pass the feedback enum (8) into set_mode.  The 251109 public
        # call stays mode=1/2; current_state is verified separately as 1/8.
        try:
            sent = bool(dr.set_mode(id_num=0, mode=int(mode)))
        except (Exception, SystemExit) as e:
            sent = False
            self.log_message.emit(
                'error', tr('Vendor set_mode(%d) failed: %s') % (mode, e))
        if not sent:
            self._closed_loop_verified = False
            return False, {jid: None for jid in target_ids}
        time.sleep(self._MODE_VERIFY_SETTLE_S)
        states = self._read_joint_states(target_ids)
        mode_verified = (sent and len(states) == len(target_ids)
                         and all(states.get(jid) == requested
                                 for jid in target_ids))

        hold_ok = True
        if mode == self._CLOSED_LOOP_MODE:
            hold_ids = [jid for jid in target_ids if jid in positions]
            hold_ok = bool(hold_ids)
            if mode_verified and hold_ids:
                hold_angles = [positions[jid] for jid in hold_ids]
                # Seed with the exact same fresh angles.  set_angles(mode=1)
                # therefore describes a zero-displacement hold; the bundled
                # 251109 preset_angle enforces the documented >=0.01 s time.
                dr.cur_angle_list = list(hold_angles)
                dr.set_angles_mode_1_flag = 1
                try:
                    hold_ok = bool(dr.set_angles(
                        id_list=list(hold_ids),
                        angle_list=list(hold_angles),
                        speed=8.0, param=40.0, mode=1))
                except (Exception, SystemExit) as e:
                    hold_ok = False
                    self.log_message.emit(
                        'error', tr('Vendor current-position hold failed: %s') % e)
            elif not mode_verified:
                hold_ok = False

        verified = bool(mode_verified and hold_ok)
        self._closed_loop_verified = bool(
            verified and mode == self._CLOSED_LOOP_MODE)
        if verified and mode == self._CLOSED_LOOP_MODE:
            self._motion_may_be_active = False
        return verified, states

    # ================= 内部辅助 =================
    def _drain_queue(self) -> None:
        """清空命令队列（queue.Queue 线程安全，GUI/工作线程均可调用）。"""
        discarded_zero: List[Tuple[int, ...]] = []
        try:
            while True:
                cmd = self._cmd_queue.get_nowait()
                if cmd and cmd[0] == 'zero' and len(cmd) > 1:
                    discarded_zero.append(tuple(cmd[1]))
        except queue.Empty:
            pass
        with self._trajectory_point_lock:
            self._pending_trajectory_point = None
        for joint_ids in discarded_zero:
            reason = tr('Set-zero command cancelled because the command queue was cleared')
            self.log_message.emit('warn', reason)
            self._finish_zero_setting(
                list(joint_ids), [], list(joint_ids), reason)

    def _finish_zero_setting(
            self, requested: List[int], succeeded: List[int],
            failed: List[int], reason: str = '') -> None:
        """Clear the zero-write latch once and publish a structured result."""
        with self._zero_setting_lock:
            if not self._zero_setting_active:
                return
            active_ids = self._zero_setting_ids
            # Only the command owning the current latch may clear it.  This is
            # defensive against a stale worker from a prior backend session.
            if active_ids and tuple(requested) != active_ids:
                return
            self._zero_setting_active = False
            self._zero_setting_ids = ()
        result = {
            'requested': list(requested),
            'succeeded': list(dict.fromkeys(succeeded)),
            'failed': list(dict.fromkeys(failed)),
            'reason': str(reason or ''),
        }
        try:
            self.zero_setting_changed.emit(False, result)
        except RuntimeError:
            pass

    def _zero_target_state_error(self, joint_ids: List[int]) -> str:
        """Return a localized rejection reason for unsafe zero-write state."""
        state = self._last_state
        try:
            timestamp = float(state.timestamp) if state is not None else math.nan
        except (TypeError, ValueError):
            timestamp = math.nan
        max_age = max(
            self._ZERO_STATE_MIN_MAX_AGE_S,
            3.0 / max(float(self._poll_hz), 0.5))
        age = time.time() - timestamp
        if (state is None or not math.isfinite(timestamp)
                or not math.isfinite(age) or abs(age) > max_age):
            return tr('Set-zero rejected: telemetry snapshot is missing or stale')

        invalid: List[int] = []
        moving: Dict[int, float] = {}
        for jid in joint_ids:
            reading = state.joint(jid)
            try:
                angle = (float(reading.angle_deg)
                         if reading is not None else math.nan)
            except (TypeError, ValueError):
                angle = math.nan
            if (reading is None or not reading.online
                    or not math.isfinite(angle)):
                invalid.append(jid)
                continue
            try:
                speed = float(reading.speed_rpm)
            except (TypeError, ValueError):
                speed = math.nan
            # Some firmwares do not publish speed (NaN).  A fresh valid angle
            # still permits the operation; a finite non-stationary speed does not.
            if (math.isfinite(speed)
                    and abs(speed) > self._ZERO_STATIONARY_SPEED_RPM):
                moving[jid] = round(speed, 3)
        if invalid:
            return tr('Set-zero rejected: no fresh valid telemetry for arm '
                      'joint IDs %s') % invalid
        if moving:
            return tr('Set-zero rejected: selected arm joints are still moving %s') % moving
        return ''

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
    'Trajectory point rejected: bandwidth must be positive': '轨迹点被拒绝：滤波带宽必须大于 0',
    'Trajectory command rejected: backend not connected': '轨迹命令被拒绝：后端未连接',
    'Trajectory command rejected: motion not enabled': '轨迹命令被拒绝：运动未使能',
    'Trajectory command rejected: empty targets': '轨迹命令被拒绝：目标为空',
    'Trajectory command rejected: unknown joint ID %s': '轨迹命令被拒绝：未知关节 ID %s',
    'Trajectory command rejected: non-finite target': '轨迹命令被拒绝：目标值无效',
    'Trajectory command rejected: %s': '轨迹命令被拒绝：%s',
    'Trajectory gripper sample ignored: gripper not configured': '夹爪未配置，已忽略轨迹中的夹爪样本',
    'Motion enable auto-disabled after E-stop; re-enable to move again': '急停后运动使能已自动关闭，需重新使能才能运动',
    '!!! E-stop request submitted (command queue cleared) !!!': '!!! 急停请求已提交（命令队列已清空）!!!',
    'E-stop request: backend not connected, cannot send to bus!': '急停请求：后端未连接，无法发送到总线！',
    'Set-zero rejected: explicit arm joint IDs are required':
        '设零点被拒绝：必须明确指定机械臂关节 ID',
    'Set-zero rejected: invalid joint ID list %s':
        '设零点被拒绝：关节 ID 列表无效 %s',
    'Set-zero rejected: no arm joints selected': '设零点被拒绝：未选择机械臂关节',
    'Set-zero rejected: duplicate joint IDs %s': '设零点被拒绝：关节 ID 重复 %s',
    'Set-zero rejected: IDs %s are not configured arm joints':
        '设零点被拒绝：ID %s 不是已配置的机械臂关节',
    'Set-zero rejected: backend not connected': '设零点被拒绝：后端未连接',
    'Set-zero rejected: motion not enabled (this operation briefly unloads joints)':
        '设零点被拒绝：运动未使能（该操作会短暂卸载关节）',
    'Set-zero rejected: another zero-setting operation is active':
        '设零点被拒绝：另一个设零操作正在进行',
    'Set-zero rejected: command queue is not idle':
        '设零点被拒绝：命令队列尚未空闲',
    'Set-zero rejected: telemetry snapshot is missing or stale':
        '设零点被拒绝：遥测快照缺失或已过期',
    'Set-zero rejected: no fresh valid telemetry for arm joint IDs %s':
        '设零点被拒绝：机械臂关节 ID %s 没有新鲜有效的遥测',
    'Set-zero rejected: selected arm joints are still moving %s':
        '设零点被拒绝：所选机械臂关节仍在运动 %s',
    'Set-zero command queued for arm joint IDs %s '
    '(persistent; joints will unload briefly)':
        '机械臂关节 ID %s 的设零命令已入队（掉电保存，关节会短暂卸载）',
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
    'Motion not enabled; discarding queued trajectory command': '运动未使能，丢弃已入队的轨迹命令',
    'Target angles sent (°): %s': '已下发目标角度(°): %s',
    'Failed to send target angles (library returned False)': '下发目标角度失败（库返回 False）',
    'Teach trajectory moving to first point (°): %s': '示教轨迹正移动到首点(°)：%s',
    'Failed to move teach trajectory to first point': '示教轨迹移动到首点失败',
    'Failed to send streamed trajectory point': '流式轨迹点下发失败',
    'Cannot hold stopped trajectory: no valid live pose': '停止轨迹后无法保持：没有有效实时位姿',
    'Teach trajectory stopped and current pose is held': '示教轨迹已停止，并保持在当前位置',
    'Failed to hold pose after stopping teach trajectory': '停止示教轨迹后保持当前位置失败',
    'Motion not enabled; discarding queued set-zero command': '运动未使能，丢弃已入队的设零点命令',
    'Set-zero cancelled: teach trajectory is active': '设零点已取消：示教轨迹正在运行',
    'Set-zero interrupted before all selected joints were written':
        '设零点被中断：尚未写完全部已选关节',
    'Set zero failed for arm joint ID %d: %s':
        '机械臂关节 ID %d 设零失败：%s',
    'Arm joint ID %d current position set as zero (persisted)':
        '机械臂关节 ID %d 的当前位置已设为零点（掉电保存）',
    'Set zero failed for arm joint ID %d (library returned False)':
        '机械臂关节 ID %d 设零失败（库返回 False）',
    'Set zero completed for arm joint IDs %s':
        '机械臂关节 ID %s 设零完成',
    'Set zero finished with failures; succeeded=%s, failed=%s':
        '设零结束但有失败项；成功=%s，失败=%s',
    'Set-zero command cancelled because the command queue was cleared':
        '设零命令已取消：命令队列被清空',
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
    'At least one arm joint ID is required': '至少需要配置一个机械臂关节 ID',
    'Arm joint IDs must be unique: %s': '机械臂关节 ID 不得重复：%s',
    'Arm joint IDs must be in [1, 63]: %s': '机械臂关节 ID 必须在 [1, 63] 范围内：%s',
    'Gripper ID must be 0 (disabled) or in [1, 63]':
        '夹爪 ID 必须为 0（禁用）或在 [1, 63] 范围内',
    'Gripper ID %d duplicates an arm joint ID': '夹爪 ID %d 与机械臂关节 ID 重复',
    '%s rejected: teach trajectory %d is active': '%s 被拒绝：示教轨迹 %d 正在运行',
    '%s rejected: arm zero-setting operation is active':
        '%s 被拒绝：机械臂设零操作正在进行',
    'CAN worker vendor call failed: %s': 'CAN 工作线程调用厂商库失败：%s',
    'J1-J4 drive error preflight failed: %s': 'J1~J4 驱动错误预检失败：%s',
    'Unable to read fresh pose for joint hold: %s': '读取关节保持所需的新鲜位姿失败：%s',
    'Joint hold rejected: fresh position missing for IDs %s':
        '关节保持被拒绝：多次采集后 ID %s 仍缺少新鲜位置',
    'Joint lock rejected: current position missing for IDs %s':
        '关节锁定被拒绝：ID %s 缺少当前位置',
    'Vendor set_mode(%d) failed: %s': '厂商 set_mode(%d) 调用失败：%s',
    'Vendor current-position hold failed: %s': '厂商当前位置保持命令失败：%s',
    'Joints safely locked and held before serial close':
        '串口关闭前已安全锁定关节并保持当前位置',
    'Final joint lock/hold before serial close failed: %s':
        '串口关闭前最后一次关节锁定/保持失败：%s',
    'Failed to hold pose after stopping teach trajectory: %s':
        '停止示教轨迹后保持当前位置失败：%s',
})
