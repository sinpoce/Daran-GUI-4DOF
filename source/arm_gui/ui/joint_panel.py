# -*- coding: utf-8 -*-
"""中央关节面板：每关节一张卡片（2x2 网格）+ 全局运动参数行。

安全约定：滑块/输入框只改目标值不发送；发送必须点【到位】或
【全部移动到目标】，且这些按钮只有在「已连接 + 运动已使能」时可用。
实际下发前 MainWindow 还会调用 backend.validate_targets() 二次校验。

信号：
    move_requested(angles: dict{joint_id: deg}, speed_rpm: float, accel: float)
"""
from __future__ import annotations

import math
from typing import Dict, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

from ..core import kinematics
from ..core.config import JointConfig, SafetyConfig
from ..core.state import ArmState, JointReading
from ..i18n import get_language, register, tr
from .style import mono_font
from .widgets import Sparkline, repolish

# config.json 的 name 字段可能是中文旧数据：已知关节名的双向映射表
_JOINT_NAME_ZH2EN = {
    'J1 底座偏航': 'J1 Base Yaw',
    'J2 肩部俯仰': 'J2 Shoulder Pitch',
    'J3 肘部俯仰': 'J3 Elbow Pitch',
    'J4 腕部俯仰': 'J4 Wrist Pitch',
}
_JOINT_NAME_EN2ZH = {en: zh for zh, en in _JOINT_NAME_ZH2EN.items()}


def display_joint_name(name: str) -> str:
    """按当前语言显示配置中的关节名：已知名字双向映射，未知名字原样显示。"""
    if get_language() == 'zh':
        return _JOINT_NAME_EN2ZH.get(name, name)
    return _JOINT_NAME_ZH2EN.get(name, name)


class JointCard(QFrame):
    """单关节卡片：读数显示 + 火花线 + 目标角度设定 + 相对步进点动。"""

    move_requested = Signal(int, float)     # (joint_id, target_deg)
    jog_requested = Signal(int, float)      # (joint_id, delta_deg 相对当前角度)

    def __init__(self, jcfg: JointConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.joint_id = jcfg.joint_id
        self._syncing = False               # 滑块/输入框互同步防递归
        self._offline = None                # None 强制首次刷新样式

        self.setObjectName('jointCard')
        self.setProperty('offline', True)

        # ---- 标题行：关节名 + ID 徽章 ----
        name_lbl = QLabel(display_joint_name(jcfg.name))
        f = name_lbl.font()
        f.setBold(True)
        name_lbl.setFont(f)
        badge = QLabel('ID %d' % jcfg.joint_id)
        badge.setObjectName('idBadge')
        head = QHBoxLayout()
        head.setSpacing(6)
        head.addWidget(name_lbl)
        head.addStretch(1)
        head.addWidget(badge)

        # ---- 当前角度（大号等宽） ----
        self.angle_lbl = QLabel('--')
        self.angle_lbl.setObjectName('angleValue')
        self.angle_lbl.setFont(mono_font(24, bold=True))
        self.angle_lbl.setAlignment(Qt.AlignmentFlag.AlignRight |
                                    Qt.AlignmentFlag.AlignVCenter)
        unit_lbl = QLabel('°')
        unit_lbl.setObjectName('dimLabel')
        unit_lbl.setFont(mono_font(12))
        angle_row = QHBoxLayout()
        angle_row.addStretch(1)
        angle_row.addWidget(self.angle_lbl)
        angle_row.addWidget(unit_lbl, 0, Qt.AlignmentFlag.AlignBottom)

        # ---- 转速 / 力矩（小字等宽） ----
        self.sub_lbl = QLabel('--  r/min    --  Nm')
        self.sub_lbl.setObjectName('dimLabel')
        self.sub_lbl.setFont(mono_font(8))
        self.sub_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)

        # ---- 火花线（最近 10 秒角度） ----
        self.spark = Sparkline(window_s=10.0)

        # ---- 分隔线 ----
        line = QFrame()
        line.setObjectName('cardLine')
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFixedHeight(1)

        # ---- 目标角度：数字框（只改值不发送，点「到位」才执行） ----
        lo, hi = jcfg.min_deg, jcfg.max_deg
        tgt_lbl = QLabel(tr('Target'))
        tgt_lbl.setObjectName('dimLabel')
        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(2)
        self.spin.setSingleStep(0.5)
        self.spin.setSuffix('°')
        self.spin.setFont(mono_font(9))
        # 最小宽度须容下 "-720.00°" + 16px 箭头按钮；固定 90px 会裁字
        self.spin.setMinimumWidth(104)
        self.btn_go = QPushButton(tr('Go'))
        self.btn_go.setMinimumWidth(44)
        self.btn_go.setToolTip(
            tr('Move this joint to the target angle (motion enable required)'))
        self.btn_go.setEnabled(False)

        self.btn_go.clicked.connect(
            lambda: self.move_requested.emit(self.joint_id, self.target()))

        tgt_row = QHBoxLayout()
        tgt_row.setSpacing(6)
        tgt_row.addWidget(tgt_lbl)
        tgt_row.addStretch(1)
        tgt_row.addWidget(self.spin)
        tgt_row.addWidget(self.btn_go)

        # ---- 相对步进点动：[−] 步进角 [+]（相对当前角度移动） ----
        jog_lbl = QLabel(tr('Jog'))
        jog_lbl.setObjectName('dimLabel')
        self.jog_step = QDoubleSpinBox()
        # 默认 0（防误触：0 步进点击无动作），需要时自行输入步进角
        self.jog_step.setRange(0.0, 90.0)
        self.jog_step.setDecimals(1)
        self.jog_step.setValue(0.0)
        self.jog_step.setSingleStep(1.0)
        self.jog_step.setSuffix('°')
        self.jog_step.setFont(mono_font(9))
        self.jog_step.setMinimumWidth(80)
        self.jog_step.setToolTip(tr('Jog step size'))
        self.btn_jog_neg = QPushButton('−')
        self.btn_jog_pos = QPushButton('+')
        for b, sign in ((self.btn_jog_neg, -1.0), (self.btn_jog_pos, +1.0)):
            b.setMinimumWidth(36)
            b.setFont(mono_font(11, bold=True))
            b.setEnabled(False)
            b.setToolTip(tr('Jog this joint by the step relative to its '
                            'current angle (motion enable required)'))
            b.clicked.connect(lambda _c=False, s=sign: self._emit_jog(s))

        jog_row = QHBoxLayout()
        jog_row.setSpacing(6)
        jog_row.addWidget(jog_lbl)
        jog_row.addStretch(1)
        jog_row.addWidget(self.btn_jog_neg)
        jog_row.addWidget(self.jog_step)
        jog_row.addWidget(self.btn_jog_pos)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(4)
        lay.addLayout(head)
        lay.addLayout(angle_row)
        lay.addWidget(self.sub_lbl)
        lay.addWidget(self.spark)
        lay.addWidget(line)
        lay.addLayout(tgt_row)
        lay.addLayout(jog_row)

        self._set_offline(True)

    def _emit_jog(self, sign: float) -> None:
        step = float(self.jog_step.value())
        if step <= 0.0:
            return          # 步进为 0 时点击无动作（防误触）
        self.jog_requested.emit(self.joint_id, sign * step)

    # ---- 对外接口 ----
    def target(self) -> float:
        return float(self.spin.value())

    def set_target(self, deg: float) -> None:
        self.spin.setValue(deg)     # 自动被 spin 范围钳位

    def set_motion_allowed(self, allowed: bool) -> None:
        self.btn_go.setEnabled(allowed)
        self.btn_jog_neg.setEnabled(allowed)
        self.btn_jog_pos.setEnabled(allowed)

    def update_reading(self, r: Optional[JointReading]) -> None:
        if r is not None and r.valid:
            self.angle_lbl.setText('%.2f' % r.angle_deg)
            spd = '--' if math.isnan(r.speed_rpm) else '%+.1f' % r.speed_rpm
            trq = '--' if math.isnan(r.torque_nm) else '%+.2f' % r.torque_nm
            self.sub_lbl.setText('%s r/min    %s Nm' % (spd, trq))
            self.spark.append(r.angle_deg)
            self._set_offline(False)
        else:
            self.angle_lbl.setText('--')
            self.sub_lbl.setText('--  r/min    --  Nm')
            self._set_offline(True)

    def _set_offline(self, offline: bool) -> None:
        if offline == self._offline:
            return
        self._offline = offline
        self.setProperty('offline', offline)
        repolish(self)


class JointPanel(QWidget):
    """关节卡片网格 + 全局运动参数行 + XYZ 点位移动栏。"""

    move_requested = Signal(object, float, float)   # (dict{id: deg}, speed, accel)
    jog_requested = Signal(int, float)              # (joint_id, delta_deg)
    cartesian_requested = Signal(float, float, float)   # X,Y,Z[mm]（Pitch 保持当前）
    gripper_move_requested = Signal(float)          # 夹爪电机目标角（绝对角）
    gripper_zero_requested = Signal()               # 设当前为夹爪闭合零点

    def __init__(self, joints, safety: SafetyConfig,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._last_state: Optional[ArmState] = None
        self._connected = False
        self._motion_allowed = False

        # ---- 2x2 卡片网格 ----
        self.cards: Dict[int, JointCard] = {}
        grid = QGridLayout()
        grid.setSpacing(10)
        for i, jcfg in enumerate(joints):
            card = JointCard(jcfg)
            card.move_requested.connect(self._on_card_move)
            card.jog_requested.connect(self.jog_requested)
            self.cards[jcfg.joint_id] = card
            grid.addWidget(card, i // 2, i % 2)
        # ---- 全局行：速度 / 加速度 / 批量操作 ----
        spd_lbl = QLabel(tr('Speed'))
        spd_lbl.setObjectName('dimLabel')
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.5, safety.max_speed_rpm)
        self.speed_spin.setDecimals(1)
        self.speed_spin.setValue(min(safety.default_speed_rpm, safety.max_speed_rpm))
        self.speed_spin.setSuffix(' r/min')
        self.speed_spin.setFont(mono_font(9))
        self.speed_spin.setToolTip(
            tr('Trapezoidal trajectory target speed (max %.1f r/min)')
            % safety.max_speed_rpm)

        acc_lbl = QLabel(tr('Acceleration'))
        acc_lbl.setObjectName('dimLabel')
        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(1.0, safety.max_accel)
        self.accel_spin.setDecimals(1)
        self.accel_spin.setValue(min(safety.default_accel, safety.max_accel))
        self.accel_spin.setFont(mono_font(9))
        self.accel_spin.setToolTip(
            tr('Trapezoidal trajectory acceleration (max %.1f)')
            % safety.max_accel)

        self.btn_copy = QPushButton(tr('Read Current → Target'))
        self.btn_copy.setToolTip(tr('Set all joint targets to their current angles'))
        self.btn_copy.setEnabled(False)
        self.btn_copy.clicked.connect(self.copy_current_to_targets)

        self.btn_move_all = QPushButton(tr('Move All to Targets'))
        self.btn_move_all.setObjectName('primaryButton')
        self.btn_move_all.setToolTip(
            tr('Run trapezoidal trajectories on all joints to their targets '
               '(motion enable required)'))
        self.btn_move_all.setEnabled(False)
        self.btn_move_all.clicked.connect(self._on_move_all)

        bar = QHBoxLayout()
        bar.setSpacing(8)
        bar.addWidget(spd_lbl)
        bar.addWidget(self.speed_spin)
        bar.addSpacing(8)
        bar.addWidget(acc_lbl)
        bar.addWidget(self.accel_spin)
        bar.addStretch(1)
        bar.addWidget(self.btn_copy)
        bar.addWidget(self.btn_move_all)

        # ---- XYZ 点位移动栏：输入末端坐标，经解析逆解直接移动（Pitch 保持当前）----
        cart_lbl = QLabel(tr('Point Move'))
        cart_lbl.setObjectName('dimLabel')
        self.cart_spins = {}
        cart_bar = QHBoxLayout()
        cart_bar.setSpacing(6)
        cart_bar.addWidget(cart_lbl)
        for key in ('X', 'Y', 'Z'):
            lbl = QLabel(key)
            lbl.setObjectName('dimLabel')
            sp = QDoubleSpinBox()
            # 范围须容下夹爪TCP最大伸展（臂382 + 工具199 ≈ 581mm）
            sp.setRange(-600.0, 600.0)
            sp.setDecimals(1)
            sp.setSuffix(' mm')
            sp.setFont(mono_font(9))
            sp.setMinimumWidth(92)
            self.cart_spins[key] = sp
            cart_bar.addWidget(lbl)
            cart_bar.addWidget(sp)
        cart_bar.addStretch(1)
        self.btn_read_pose = QPushButton(tr('Read Pose'))
        self.btn_read_pose.setToolTip(
            tr('Fill the point fields from the current end-effector pose'))
        self.btn_read_pose.setEnabled(False)
        self.btn_read_pose.clicked.connect(self._on_read_pose)
        self.btn_cart_move = QPushButton(tr('Move to Point'))
        self.btn_cart_move.setObjectName('primaryButton')
        self.btn_cart_move.setToolTip(
            tr('Move the end-effector to the given XYZ via analytic IK, '
               'keeping the current pitch (motion enable required)'))
        self.btn_cart_move.setEnabled(False)
        self.btn_cart_move.clicked.connect(self._on_cart_move)
        cart_bar.addWidget(self.btn_read_pose)
        cart_bar.addWidget(self.btn_cart_move)

        # ---- 夹爪控制栏：当前角度 + 目标角度 + 闭合置零 ----
        grip_lbl = QLabel(tr('Gripper'))
        grip_lbl.setObjectName('dimLabel')
        self.grip_cur = QLabel('--')
        self.grip_cur.setFont(mono_font(11, bold=True))
        self.grip_cur.setToolTip(tr('Current gripper motor angle'))
        self.grip_spin = QDoubleSpinBox()
        self.grip_spin.setRange(-720.0, 720.0)
        self.grip_spin.setDecimals(1)
        self.grip_spin.setSingleStep(1.0)
        self.grip_spin.setSuffix('°')
        self.grip_spin.setFont(mono_font(9))
        self.grip_spin.setMinimumWidth(96)
        self.btn_grip_go = QPushButton(tr('Set Gripper Angle'))
        self.btn_grip_go.setToolTip(
            tr('Move the gripper motor to the given angle '
               '(0° = closed zero; motion enable required)'))
        self.btn_grip_go.setEnabled(False)
        self.btn_grip_go.clicked.connect(
            lambda: self.gripper_move_requested.emit(float(self.grip_spin.value())))
        self.btn_grip_zero = QPushButton(tr('Set Closed Zero'))
        self.btn_grip_zero.setToolTip(
            tr('Set the current gripper position as the closed zero point '
               '(persisted; close the gripper first)'))
        self.btn_grip_zero.setEnabled(False)
        self.btn_grip_zero.clicked.connect(self.gripper_zero_requested)

        grip_bar = QHBoxLayout()
        grip_bar.setSpacing(6)
        grip_bar.addWidget(grip_lbl)
        grip_bar.addWidget(self.grip_cur)
        grip_bar.addStretch(1)
        grip_bar.addWidget(self.grip_spin)
        grip_bar.addWidget(self.btn_grip_go)
        grip_bar.addWidget(self.btn_grip_zero)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)
        lay.addLayout(grid, 1)
        lay.addLayout(bar, 0)
        lay.addLayout(cart_bar, 0)
        lay.addLayout(grip_bar, 0)

    # ---- 状态刷新（来自 backend.state_updated，已在 GUI 线程） ----
    def update_state(self, state: ArmState) -> None:
        self._last_state = state
        for jid, card in self.cards.items():
            card.update_reading(state.joint(jid))
        g = state.gripper
        if g is not None and g.valid:
            self.grip_cur.setText('%.2f°' % g.angle_deg)
        else:
            self.grip_cur.setText('--')

    def clear_readings(self) -> None:
        """断开后：卡片全部转灰显示 '--'。"""
        self._last_state = None
        for card in self.cards.values():
            card.update_reading(None)
            card.spark.clear()
        self.grip_cur.setText('--')

    # ---- 使能/连接联动 ----
    def set_connected(self, connected: bool) -> None:
        self._connected = connected
        self.btn_copy.setEnabled(connected)
        self.btn_read_pose.setEnabled(connected)
        if not connected:
            self.set_motion_allowed(False)

    def set_motion_allowed(self, allowed: bool) -> None:
        self._motion_allowed = allowed and self._connected
        self.btn_move_all.setEnabled(self._motion_allowed)
        self.btn_cart_move.setEnabled(self._motion_allowed)
        self.btn_grip_go.setEnabled(self._motion_allowed)
        self.btn_grip_zero.setEnabled(self._motion_allowed)
        for card in self.cards.values():
            card.set_motion_allowed(self._motion_allowed)

    # ---- 运动参数 ----
    def speed_rpm(self) -> float:
        return float(self.speed_spin.value())

    def accel(self) -> float:
        return float(self.accel_spin.value())

    # ---- 目标操作 ----
    def copy_current_to_targets(self) -> None:
        if self._last_state is None:
            return
        for jid, card in self.cards.items():
            r = self._last_state.joint(jid)
            if r is not None and r.valid:
                card.set_target(r.angle_deg)

    def all_targets(self) -> Dict[int, float]:
        return {jid: card.target() for jid, card in self.cards.items()}

    # ---- 点位移动 ----
    def _current_motors(self):
        """当前 4 关节电机角（ID 1~4 全在线才返回，否则 None）。"""
        if self._last_state is None:
            return None
        motors = []
        for jid in (1, 2, 3, 4):
            r = self._last_state.joint(jid)
            if r is None or not r.valid:
                return None
            motors.append(r.angle_deg)
        return motors

    def _on_read_pose(self) -> None:
        motors = self._current_motors()
        if motors is None:
            return
        x, y, z, _pitch = kinematics.fk_from_motor(motors)
        self.cart_spins['X'].setValue(x)
        self.cart_spins['Y'].setValue(y)
        self.cart_spins['Z'].setValue(z)

    def _on_cart_move(self) -> None:
        self.cartesian_requested.emit(
            float(self.cart_spins['X'].value()),
            float(self.cart_spins['Y'].value()),
            float(self.cart_spins['Z'].value()))

    # ---- 内部：发信号（实际校验与下发由 MainWindow 完成） ----
    def _on_card_move(self, joint_id: int, target: float) -> None:
        self.move_requested.emit({joint_id: target},
                                 self.speed_rpm(), self.accel())

    def _on_move_all(self) -> None:
        self.move_requested.emit(self.all_targets(),
                                 self.speed_rpm(), self.accel())


register({
    'Target': '目标',
    'Go': '到位',
    'Jog': '点动',
    'Jog step size': '点动步进角',
    'Jog this joint by the step relative to its current angle '
    '(motion enable required)':
        '按步进角相对当前角度点动该关节（需使能运动）',
    'Point Move': '点位移动',
    'Read Pose': '读取位姿',
    'Fill the point fields from the current end-effector pose':
        '把当前末端位姿填入点位输入框',
    'Move to Point': '移动到点位',
    'Move the end-effector to the given XYZ via analytic IK, '
    'keeping the current pitch (motion enable required)':
        '通过解析逆解把末端移动到给定 XYZ，Pitch 保持当前值（需使能运动）',
    'Gripper': '夹爪',
    'Current gripper motor angle': '夹爪电机当前角度',
    'Set Gripper Angle': '设定夹爪角度',
    'Move the gripper motor to the given angle '
    '(0° = closed zero; motion enable required)':
        '把夹爪电机转到给定角度（0°=闭合零点；需使能运动）',
    'Set Closed Zero': '设为闭合零点',
    'Set the current gripper position as the closed zero point '
    '(persisted; close the gripper first)':
        '把夹爪当前位置设为闭合零点（掉电保存；请先把夹爪闭合到位）',
    'Move this joint to the target angle (motion enable required)':
        '将该关节移动到目标角度（需使能运动）',
    'Speed': '速度',
    'Acceleration': '加速度',
    'Trapezoidal trajectory target speed (max %.1f r/min)':
        '梯形轨迹目标转速（上限 %.1f r/min）',
    'Trapezoidal trajectory acceleration (max %.1f)':
        '梯形轨迹加速度参数（上限 %.1f）',
    'Read Current → Target': '读取当前→目标',
    'Set all joint targets to their current angles':
        '把所有关节的目标角度设置为当前角度',
    'Move All to Targets': '全部移动到目标',
    'Run trapezoidal trajectories on all joints to their targets '
    '(motion enable required)':
        '全部关节按目标角度执行梯形轨迹（需使能运动）',
})
