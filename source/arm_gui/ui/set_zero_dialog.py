# -*- coding: utf-8 -*-
"""Dedicated UI for persistently zeroing selected arm joints.

This module never writes to a drive directly.  It presents the latest J1-J4
telemetry, applies the local UI gate, and emits the deliberately selected IDs.
The main window owns the final confirmation and backend safety checks.
"""
from __future__ import annotations

import math
import time
from typing import Callable, Optional, Sequence

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QDialog, QFrame, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QPushButton, QScrollArea, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from ..core.state import ArmState
from ..i18n import register, tr


class SetZeroDialog(QDialog):
    """Select configured arm joints and request a persistent zero write."""

    _TELEMETRY_FRESH_S = 1.0
    _STATIONARY_SPEED_RPM = 0.5
    set_zero_requested = Signal(object)  # list[int]

    def __init__(
        self,
        *,
        joints: Sequence[tuple[int, str]],
        get_connected: Callable[[], bool],
        get_motion_enabled: Callable[[], bool],
        get_teach_busy: Callable[[], bool],
        get_zero_busy: Callable[[], bool],
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle(tr('Set Zero'))
        self.resize(720, 640)
        self.setMinimumSize(500, 400)
        self.setModal(False)

        # ``cfg.joints`` is passed by MainWindow; the separately configured
        # gripper is deliberately never included in this operation.
        self._joints = tuple(
            (int(jid), str(name or ('J%d' % int(jid))))
            for jid, name in joints)
        self._joint_ids = tuple(jid for jid, _name in self._joints)
        self._get_connected = get_connected
        self._get_motion_enabled = get_motion_enabled
        self._get_teach_busy = get_teach_busy
        self._get_zero_busy = get_zero_busy
        self._zero_progress_text = ''
        self._zero_result_text = ''
        self._last_valid_monotonic: dict[int, float] = {
            jid: 0.0 for jid in self._joint_ids}

        self.joint_checks: dict[int, QCheckBox] = {}
        self._angle_items: dict[int, QTableWidgetItem] = {}
        self._status_items: dict[int, QTableWidgetItem] = {}

        self._build_ui()
        self._freshness_timer = QTimer(self)
        self._freshness_timer.setInterval(250)
        self._freshness_timer.timeout.connect(self.update_control_state)
        self._freshness_timer.start()
        self.update_control_state()

    def _build_ui(self) -> None:
        heading = QLabel(tr('Set Selected Joint Zero Points'))
        font = heading.font()
        font.setPointSize(font.pointSize() + 3)
        font.setBold(True)
        heading.setFont(font)

        explanation = QLabel(tr(
            'This tool does not move the arm. It redefines each selected arm '
            'joint\'s current position as 0°. The new zero point is stored in '
            'the drive and remains after power cycling. The gripper is not '
            'included.'))
        explanation.setWordWrap(True)

        warning = QLabel(tr(
            'Warning: a selected joint may briefly lose holding torque while '
            'its zero point is written. Mechanically support the arm and place '
            'every selected joint at its intended zero position first.'))
        warning.setWordWrap(True)
        warning.setObjectName('warningLabel')

        coordinates = QLabel(tr(
            'Changing a joint zero changes the coordinate reference. Existing '
            'teach recordings, calibration data, and saved points may no '
            'longer reproduce correctly.'))
        coordinates.setWordWrap(True)
        coordinates.setObjectName('dimLabel')

        readings_box = QGroupBox(tr('Current Joint Positions'))
        readings_layout = QVBoxLayout(readings_box)
        self.table = QTableWidget(len(self._joints), 5)
        self.table.setHorizontalHeaderLabels([
            tr('Select'), tr('Joint'), tr('ID'), tr('Current Angle'),
            tr('Online Status'),
        ])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(170)
        self.table.verticalHeader().setDefaultSectionSize(32)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for column in (2, 3, 4):
            header.setSectionResizeMode(
                column, QHeaderView.ResizeMode.ResizeToContents)

        for row, (jid, name) in enumerate(self._joints):
            checkbox = QCheckBox()
            checkbox.setEnabled(False)  # enabled by fresh, valid telemetry
            checkbox.setToolTip(tr('Select J%d for zero setting') % jid)
            checkbox.stateChanged.connect(
                lambda _state, self=self: self.update_control_state())
            check_cell = QWidget()
            check_layout = QHBoxLayout(check_cell)
            check_layout.setContentsMargins(6, 0, 6, 0)
            check_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            check_layout.addWidget(checkbox)
            self.table.setCellWidget(row, 0, check_cell)
            self.joint_checks[jid] = checkbox

            name_item = QTableWidgetItem(name)
            id_item = QTableWidgetItem(str(jid))
            id_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            angle_item = QTableWidgetItem(tr('No telemetry'))
            angle_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            status_item = QTableWidgetItem(tr('No telemetry'))
            status_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table.setItem(row, 1, name_item)
            self.table.setItem(row, 2, id_item)
            self.table.setItem(row, 3, angle_item)
            self.table.setItem(row, 4, status_item)
            self._angle_items[jid] = angle_item
            self._status_items[jid] = status_item

        self.btn_select_all = QPushButton(tr('Select All Arm Joints'))
        self.btn_select_all.setEnabled(False)
        self.btn_select_all.clicked.connect(self.select_all)
        self.btn_clear_selection = QPushButton(tr('Clear Selection'))
        self.btn_clear_selection.clicked.connect(self.clear_selection)
        selection_bar = QHBoxLayout()
        selection_bar.addWidget(self.btn_select_all)
        selection_bar.addWidget(self.btn_clear_selection)
        selection_bar.addStretch(1)

        readings_layout.addWidget(self.table)
        readings_layout.addLayout(selection_bar)

        self.status_label = QLabel('')
        self.status_label.setWordWrap(True)
        self.status_label.setObjectName('statusPanelLabel')

        self.btn_set_selected_zero = QPushButton(tr('Set Selected to Zero'))
        self.btn_set_selected_zero.setObjectName('primaryButton')
        self.btn_set_selected_zero.setToolTip(tr(
            'Persist each selected joint\'s current position as 0°'))
        self.btn_set_selected_zero.clicked.connect(self._request_set_zero)

        close_button = QPushButton(tr('Close'))
        close_button.clicked.connect(self.close)
        buttons = QHBoxLayout()
        buttons.addStretch(1)
        buttons.addWidget(close_button)
        buttons.addWidget(self.btn_set_selected_zero)

        # Keep the full safety explanation and all controls reachable on
        # lower-resolution/high-DPI PCs.  At the normal 720x640 size no
        # scrollbar is shown; if the dialog must shrink, the content scrolls
        # instead of letting the table and selection buttons overlap.
        content = QWidget()
        content.setObjectName('dialogContent')
        content.setMinimumHeight(600)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(10)
        layout.addWidget(heading)
        layout.addWidget(explanation)
        layout.addWidget(warning)
        layout.addWidget(coordinates)
        layout.addWidget(readings_box, 1)
        layout.addWidget(self.status_label)
        layout.addLayout(buttons)

        scroll = QScrollArea(self)
        scroll.setObjectName('dialogScrollArea')
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setWidget(content)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    @staticmethod
    def _safe_bool(callback: Callable[[], bool]) -> bool:
        """Treat an unavailable/tearing-down backend callback as not ready."""
        try:
            return bool(callback())
        except (AttributeError, RuntimeError):
            return False

    def _has_fresh_reading(self, jid: int, now: Optional[float] = None) -> bool:
        stamp = self._last_valid_monotonic.get(jid, 0.0)
        if stamp <= 0.0:
            return False
        current = time.monotonic() if now is None else now
        return current - stamp <= self._TELEMETRY_FRESH_S

    def selected_joint_ids(self) -> list[int]:
        return [
            jid for jid in self._joint_ids
            if self.joint_checks[jid].isChecked()
        ]

    def select_all(self) -> None:
        # Never silently select only the online subset: this button is usable
        # only when every configured arm joint has fresh valid telemetry.
        if not self.btn_select_all.isEnabled():
            return
        for checkbox in self.joint_checks.values():
            checkbox.setChecked(True)
        self.update_control_state()

    def clear_selection(self) -> None:
        for checkbox in self.joint_checks.values():
            checkbox.setChecked(False)
        self.update_control_state()

    def update_control_state(self) -> None:
        """Refresh row selection gates and the persistent-write gate."""
        connected = self._safe_bool(self._get_connected)
        motion_enabled = self._safe_bool(self._get_motion_enabled)
        teach_busy = self._safe_bool(self._get_teach_busy)
        zero_busy = self._safe_bool(self._get_zero_busy)
        now = time.monotonic()
        fresh = {
            jid: connected and self._has_fresh_reading(jid, now)
            for jid in self._joint_ids
        }

        for jid, checkbox in self.joint_checks.items():
            selectable = fresh[jid] and not teach_busy and not zero_busy
            if not selectable and checkbox.isChecked():
                checkbox.setChecked(False)
            checkbox.setEnabled(selectable)
            if (not fresh[jid]
                    and self._last_valid_monotonic.get(jid, 0.0) > 0.0
                    and self._status_items[jid].text() == tr('Online')):
                self._status_items[jid].setText(tr('Stale telemetry'))

        all_fresh = bool(self._joint_ids) and all(fresh.values())
        self.btn_select_all.setEnabled(
            all_fresh and not teach_busy and not zero_busy)
        selected = self.selected_joint_ids()
        ready = (connected and motion_enabled and not teach_busy
                 and not zero_busy
                 and bool(selected) and all(fresh[jid] for jid in selected))
        self.btn_set_selected_zero.setEnabled(ready)

        if zero_busy:
            status = (self._zero_progress_text
                      or tr('Setting selected zero points… Please wait.'))
        elif self._zero_result_text:
            status = self._zero_result_text
        elif teach_busy:
            status = tr('Unavailable while a teach operation is active.')
        elif not connected:
            status = tr('Not ready: connect to the arm first.')
        elif not motion_enabled:
            status = tr('Not ready: lock the joints and enable motion first.')
        elif not selected:
            status = tr('Select at least one online arm joint.')
        else:
            status = tr(
                'Ready to set zero for: %s. Verify the displayed angles first.') \
                % ', '.join('J%d' % jid for jid in selected)
        self.status_label.setText(status)

    def show_zero_setting_state(self, busy: bool, payload: object) -> None:
        """Show asynchronous backend progress/result without blocking the UI."""
        if busy:
            try:
                ids = [int(jid) for jid in payload]
            except (TypeError, ValueError):
                ids = []
            self._zero_result_text = ''
            self._zero_progress_text = (
                tr('Setting zero points for: %s. Please wait.')
                % ', '.join('J%d' % jid for jid in ids)
                if ids else tr('Setting selected zero points… Please wait.'))
        else:
            self._zero_progress_text = ''
            result = payload if isinstance(payload, dict) else {}
            succeeded = [int(jid) for jid in result.get('succeeded', [])]
            failed = [int(jid) for jid in result.get('failed', [])]
            reason = str(result.get('reason', '') or '')
            if succeeded and not failed:
                self._zero_result_text = tr(
                    'Zero setting completed for: %s.') % ', '.join(
                        'J%d' % jid for jid in succeeded)
            else:
                succeeded_text = (', '.join('J%d' % jid for jid in succeeded)
                                  or tr('None'))
                failed_text = (', '.join('J%d' % jid for jid in failed)
                               or tr('None'))
                self._zero_result_text = tr(
                    'Zero setting finished. Succeeded: %s. Failed: %s.') \
                    % (succeeded_text, failed_text)
                if reason:
                    self._zero_result_text += ' ' + tr('Reason: %s') % reason
        self.update_control_state()

    def observe_state(self, state: Optional[ArmState]) -> None:
        """Display live angle/online state for every configured arm joint."""
        if state is None:
            self.clear_readings()
            return
        now = time.monotonic()
        try:
            timestamp = float(state.timestamp)
        except (TypeError, ValueError):
            timestamp = math.nan
        state_age = time.time() - timestamp
        state_is_fresh = (math.isfinite(timestamp)
                          and math.isfinite(state_age)
                          and abs(state_age) <= self._TELEMETRY_FRESH_S)
        for jid in self._joint_ids:
            reading = state.joint(jid)
            reading_valid = (reading is not None and reading.valid
                             and math.isfinite(float(reading.angle_deg)))
            moving = (reading_valid
                      and math.isfinite(float(reading.speed_rpm))
                      and abs(float(reading.speed_rpm))
                      > self._STATIONARY_SPEED_RPM)
            valid = reading_valid and state_is_fresh and not moving
            if valid:
                self._last_valid_monotonic[jid] = now
            else:
                self._last_valid_monotonic[jid] = 0.0
            if reading is None:
                angle_text = tr('No telemetry')
                status_text = tr('No telemetry')
            elif not reading.online:
                angle_text = tr('No telemetry')
                status_text = tr('Offline')
            elif not reading_valid:
                angle_text = tr('Invalid')
                status_text = tr('Invalid telemetry')
            elif moving:
                angle_text = '%.2f°' % reading.angle_deg
                status_text = tr('Moving (%.2f r/min)') % reading.speed_rpm
            elif not state_is_fresh:
                angle_text = '%.2f°' % reading.angle_deg
                status_text = tr('Stale telemetry')
            else:
                angle_text = '%.2f°' % reading.angle_deg
                status_text = tr('Online')
            self._angle_items[jid].setText(angle_text)
            self._status_items[jid].setText(status_text)
        self.update_control_state()

    def clear_readings(self) -> None:
        for jid in self._joint_ids:
            self._last_valid_monotonic[jid] = 0.0
            checkbox = self.joint_checks[jid]
            checkbox.setChecked(False)
            checkbox.setEnabled(False)
            self._angle_items[jid].setText(tr('No telemetry'))
            self._status_items[jid].setText(tr('No telemetry'))
        self.update_control_state()

    def _request_set_zero(self) -> None:
        # Re-read all gates on the click itself; checkbox, connection, teach,
        # or telemetry freshness may have changed since the previous refresh.
        self.update_control_state()
        if self.btn_set_selected_zero.isEnabled():
            self.set_zero_requested.emit(self.selected_joint_ids())


register({
    'Set Zero': '设置零点',
    'Set Selected Joint Zero Points': '设置所选关节零点',
    'This tool does not move the arm. It redefines each selected arm joint\'s '
    'current position as 0°. The new zero point is stored in the drive and '
    'remains after power cycling. The gripper is not included.':
        '此功能不会移动机械臂，而是把每个所选机械臂关节的当前位置重新定义为 '
        '0°。新零点会保存到驱动器中，断电重启后仍然有效；夹爪不包含在此功能中。',
    'Warning: a selected joint may briefly lose holding torque while its zero '
    'point is written. Mechanically support the arm and place every selected '
    'joint at its intended zero position first.':
        '警告：写入零点时，所选关节可能短暂失去保持力矩。请先可靠支撑机械臂，'
        '并将每个所选关节摆到预期零位。',
    'Changing a joint zero changes the coordinate reference. Existing teach '
    'recordings, calibration data, and saved points may no longer reproduce '
    'correctly.':
        '修改关节零点会改变坐标基准；已有示教录制、标定数据和保存点位可能无法再'
        '正确复现。',
    'Current Joint Positions': '当前关节位置',
    'Select': '选择',
    'Joint': '关节',
    'ID': 'ID',
    'Current Angle': '当前角度',
    'Online Status': '在线状态',
    'Select J%d for zero setting': '选择 J%d 设置零点',
    'No telemetry': '无遥测数据',
    'Offline': '离线',
    'Online': '在线',
    'Stale telemetry': '遥测已过期',
    'Moving (%.2f r/min)': '运动中（%.2f r/min）',
    'Invalid': '无效',
    'Invalid telemetry': '遥测无效',
    'Select All Arm Joints': '全选机械臂关节',
    'Clear Selection': '清除选择',
    'Set Selected to Zero': '将所选关节设为零点',
    'Persist each selected joint\'s current position as 0°':
        '将每个所选关节的当前位置持久保存为 0°',
    'Close': '关闭',
    'Unavailable while a teach operation is active.':
        '示教操作进行期间不可设置零点。',
    'Setting selected zero points… Please wait.':
        '正在设置所选关节零点，请稍候。',
    'Setting zero points for: %s. Please wait.':
        '正在为以下关节设置零点：%s。请稍候。',
    'Zero setting completed for: %s.': '以下关节的零点设置已完成：%s。',
    'Zero setting finished. Succeeded: %s. Failed: %s.':
        '零点设置结束。成功：%s。失败：%s。',
    'None': '无',
    'Reason: %s': '原因：%s',
    'Not ready: connect to the arm first.': '尚未就绪：请先连接机械臂。',
    'Not ready: lock the joints and enable motion first.':
        '尚未就绪：请先锁定关节并使能运动。',
    'Select at least one online arm joint.': '请至少选择一个在线的机械臂关节。',
    'Ready to set zero for: %s. Verify the displayed angles first.':
        '已准备为以下关节设置零点：%s。请先核对显示的角度。',
})
