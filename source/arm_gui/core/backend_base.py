# -*- coding: utf-8 -*-
"""机械臂控制后端抽象基类。

两个实现：
  * CanBackend  —— 本机 USB-CAN（串口）直连，见 can_backend.py

线程模型约定
------------
* 所有公开方法都必须**非阻塞**，可从 GUI 线程直接调用；
  实际 I/O 在后端自己的工作线程里执行。
* 状态、日志、连接变化一律通过 Qt 信号发回（信号可跨线程安全 emit）。
* shutdown() 是唯一允许短暂阻塞的方法（等待线程退出），供窗口关闭时调用。

安全约定
--------
* 运动默认禁止（motion_enabled=False）；command_angles / set_zero_here
  在未使能时必须拒绝并给出日志。
* estop() 在任何状态下（包括未使能）都必须尽最大努力立即发出。
* 所有目标角度先经 validate_targets() 软限位/步长校验，拒绝整条命令而非部分执行。
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import QObject, Signal

from ..i18n import tr, register
from .state import ArmState  # noqa: F401  (契约类型，实现类使用)


class SafetyLimits:
    """软限位与运动参数上限。GUI 从配置构造后交给后端。"""

    def __init__(self,
                 angle_limits: Optional[Dict[int, Tuple[float, float]]] = None,
                 max_speed_rpm: float = 30.0,
                 max_accel: float = 50.0,
                 max_step_deg: float = 30.0):
        self.angle_limits = dict(angle_limits or {})   # {joint_id: (min_deg, max_deg)}
        self.max_speed_rpm = float(max_speed_rpm)
        self.max_accel = float(max_accel)
        self.max_step_deg = float(max_step_deg)        # 相对当前角度的单次最大步进（<=0 不限制）


class ArmBackend(QObject):
    """抽象后端。子类实现 _do_* 钩子并管理自己的工作线程。"""

    # ---- 信号（UI 只依赖这些） ----
    state_updated = Signal(object)          # ArmState
    connection_changed = Signal(bool, str)  # (connected, 描述/原因)
    log_message = Signal(str, str)          # (level: 'info'|'warn'|'error', text)
    motion_enabled_changed = Signal(bool)
    joints_mode_changed = Signal(int)       # 关节模式：1=待机(释放) 2=闭环(锁定)
    # (busy, payload): busy=True 时 payload 为本次明确选择的关节 ID；
    # busy=False 时 payload 为包含 requested/succeeded/failed/reason 的结果字典。
    # 设零会写入驱动器 Flash，耗时可达每轴约 2 秒，UI 用此信号显示进度并
    # 禁止重复提交，而不阻塞 GUI 线程。
    zero_setting_changed = Signal(bool, object)

    name = 'base'

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._motion_enabled = False
        self.safety = SafetyLimits()

    # ---- 连接管理 ----
    def start(self, params: dict) -> None:
        """异步发起连接。params 内容由具体后端定义（见各实现 docstring）。"""
        raise NotImplementedError

    def shutdown(self) -> None:
        """断开并回收线程。窗口关闭/切换后端时调用，允许短暂阻塞。"""
        raise NotImplementedError

    def is_connected(self) -> bool:
        raise NotImplementedError

    # ---- 运动使能 ----
    def set_motion_enabled(self, enabled: bool) -> None:
        raise NotImplementedError

    def is_motion_enabled(self) -> bool:
        return self._motion_enabled

    # ---- 控制命令（全部异步入队） ----
    def command_angles(self, angles_deg: Dict[int, float],
                       speed_rpm: float, accel: float,
                       allow_large_step: bool = False) -> None:
        """多关节绝对角度命令（梯形轨迹）。需先使能运动。

        allow_large_step=True 时跳过单步步长校验（软限位仍然硬性校验）——
        仅供 UI 在用户明确确认大幅移动（如回零确认框）后使用。
        """
        raise NotImplementedError

    def begin_trajectory(self) -> int:
        """Start a cancellable streamed-trajectory session.

        The returned token is passed back with every trajectory command so a
        backend can discard stale points after Stop Playback.  Backends that
        do not support streamed trajectories return ``0``.
        """
        self.log_message.emit(
            'warn', tr('Current backend does not support streamed trajectories'))
        return 0

    def command_trajectory_start(
            self, token: int, angles_deg: Dict[int, float],
            speed_rpm: float, accel: float,
            gripper_deg: Optional[float] = None) -> None:
        """Move smoothly to the first sample of a streamed trajectory."""
        self.log_message.emit(
            'warn', tr('Current backend does not support streamed trajectories'))

    def command_trajectory_point(
            self, token: int, angles_deg: Dict[int, float],
            bandwidth_hz: float,
            gripper_deg: Optional[float] = None) -> None:
        """Submit one dense trajectory point using the vendor tracking mode."""
        self.log_message.emit(
            'warn', tr('Current backend does not support streamed trajectories'))

    def cancel_trajectory(self, token: int, hold_position: bool = True) -> None:
        """Invalidate queued trajectory points and optionally hold the live pose."""
        return

    def is_trajectory_idle(self, token: int) -> bool:
        """Return True after the latest submitted point has finished sending."""
        return True

    def estop(self) -> None:
        """急停（广播）。任何状态下都可调用，优先于队列中的其他命令。"""
        raise NotImplementedError

    def set_zero_here(self, joint_ids: Optional[List[int]] = None) -> None:
        """把明确指定的机械臂关节当前位置设为持久零点。

        ``joint_ids`` 必须由调用方显式给出。实现不得用 CAN ID 0 广播，
        因为同一总线上可能还有夹爪或其他不应被设零的电机。
        """
        raise NotImplementedError

    def is_setting_zero(self) -> bool:
        """设零配置写入是否正在排队或执行。"""
        return False

    def set_joints_mode(self, mode: int) -> None:
        """切换全部关节模式（广播）：1=待机(卸载/可手推)，2=闭环(锁定保持)。

        默认实现：不支持，仅日志。可选能力，后端按需覆盖。
        """
        self.log_message.emit('warn', tr('Current backend does not support lock/release joints'))

    # ---- 夹爪（可选能力，后端按需覆盖） ----
    def gripper_command(self, angle_deg: float,
                        speed_rpm: Optional[float] = None,
                        accel: Optional[float] = None) -> None:
        """夹爪电机绝对角度命令（需使能运动）。"""
        self.log_message.emit('warn', tr('Current backend does not support the gripper'))

    def gripper_set_zero(self) -> None:
        """把夹爪当前位置设为闭合零点（掉电保存，需使能运动）。"""
        self.log_message.emit('warn', tr('Current backend does not support the gripper'))

    def set_poll_rate(self, hz: float) -> None:
        raise NotImplementedError

    # ---- 公共安全校验 ----
    def validate_targets(self, angles_deg: Dict[int, float],
                         current: Optional[ArmState],
                         check_step: bool = True) -> Tuple[bool, str]:
        """软限位 + 单次步长校验。返回 (ok, 拒绝原因)。任一超限则整条拒绝。

        check_step=False 时跳过步长校验（软限位永远硬性校验）——
        用于用户确认过的大幅移动。
        """
        for jid, target in angles_deg.items():
            lim = self.safety.angle_limits.get(jid)
            if lim is not None and not (lim[0] <= target <= lim[1]):
                return False, (tr('Joint %d target %.2f° exceeds soft limit [%.1f, %.1f]°')
                               % (jid, target, lim[0], lim[1]))
            if check_step and self.safety.max_step_deg > 0 and current is not None:
                j = current.joint(jid)
                if j is not None and j.valid:
                    step = abs(target - j.angle_deg)
                    if step > self.safety.max_step_deg:
                        return False, (tr('Joint %d single step %.1f° exceeds limit %.1f°')
                                       % (jid, step, self.safety.max_step_deg))
        return True, ''

    def clamp_motion_params(self, speed_rpm: float, accel: float) -> Tuple[float, float]:
        speed = max(0.5, min(float(speed_rpm), self.safety.max_speed_rpm))
        acc = max(1.0, min(float(accel), self.safety.max_accel))
        return speed, acc


def make_offline_state(joint_ids: List[int], names: List[str], source: str) -> ArmState:
    """构造一个全离线的初始状态（UI 初始显示用）。"""
    from .state import JointReading
    joints = [JointReading(joint_id=jid, name=(names[i] if i < len(names) else 'joint%d' % jid))
              for i, jid in enumerate(joint_ids)]
    return ArmState(joints=joints, source=source)


# ---- i18n 词条注册（英文源文案 -> 改写前中文原文）----
register({
    'Current backend does not support the gripper': '当前后端不支持夹爪',
    'Current backend does not support lock/release joints': '当前后端不支持 锁定/释放 关节',
    'Current backend does not support streamed trajectories': '当前后端不支持流式轨迹',
    'Joint %d target %.2f° exceeds soft limit [%.1f, %.1f]°': '关节%d 目标 %.2f° 超出软限位 [%.1f, %.1f]°',
    'Joint %d single step %.1f° exceeds limit %.1f°': '关节%d 单次步进 %.1f° 超过上限 %.1f°',
})
