# -*- coding: utf-8 -*-
"""Keyboard-driven manual robot point recorder.

The operator manually places the nail/TCP at the prompted target corner and
presses Enter. A short stable telemetry window is averaged, persisted, and
the prompt advances to the next point. This dialog never commands motion.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import re
import time
from collections import deque
from datetime import datetime, timezone
from typing import Callable, Optional

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout,
)

from ..core import kinematics
from ..core.apppaths import user_path
from ..core.state import ArmState
from ..core.target3d import TargetModel, default_model_path
from ..i18n import register, tr
from .widgets import repolish


DEFAULT_POINT_SEQUENCE = "1, 6, 8, 13, 15, 20, 22, 27, 29, 34, 36, 41"
STABLE_WINDOW_S = 0.8
MIN_STABLE_SAMPLES = 6
MAX_STABLE_SPEED_RPM = 0.5


def _sha256(path: str) -> Optional[str]:
    if not os.path.isfile(path):
        return None
    value = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def parse_point_sequence(text: str) -> list[int]:
    """Parse comma/space separated global target corner IDs."""
    tokens = [token for token in re.split(r"[\s,;，；]+", text.strip()) if token]
    if not tokens:
        raise ValueError(tr("Enter at least one target point ID"))
    values = [int(token) for token in tokens]
    bad = [value for value in values if not 0 <= value <= 41]
    if bad:
        raise ValueError(tr("Target point IDs must be between 0 and 41: %s") % bad)
    return values


class PointRecorderDialog(QDialog):
    """Non-modal, Enter-to-record point collection dialog."""

    def __init__(
        self,
        *,
        get_state: Callable[[], Optional[ArmState]],
        get_joints_mode: Optional[Callable[[], int]] = None,
        get_connected: Optional[Callable[[], bool]] = None,
        lock_joints: Optional[Callable[[], None]] = None,
        release_joints: Optional[Callable[[], None]] = None,
        output_root: Optional[str] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle(tr("Manual Point Recorder"))
        self.resize(920, 650)
        self.setModal(False)

        self._get_state = get_state
        self._get_joints_mode = get_joints_mode
        self._get_connected = get_connected
        self._lock_joints = lock_joints
        self._release_joints = release_joints
        self._output_root = output_root or user_path(
            "calibration", "manual_point_sessions")
        self._samples: deque[dict] = deque(maxlen=120)
        self._sequence: list[int] = []
        self._steps: list[tuple[int, int]] = []
        self._step_index = 0
        self._records: list[dict] = []
        self._session_dir: Optional[str] = None
        self._session_started_at: Optional[str] = None
        self._finished = False

        try:
            self._target_model = TargetModel(default_model_path())
        except Exception:
            self._target_model = None

        self._build_ui()
        self._enter_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Return), self)
        self._enter_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self._enter_shortcut.activated.connect(self._enter_action)
        self._numpad_enter_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Enter), self)
        self._numpad_enter_shortcut.setContext(
            Qt.ShortcutContext.WidgetWithChildrenShortcut)
        self._numpad_enter_shortcut.activated.connect(self._enter_action)
        self._update_prompt()

    def _build_ui(self) -> None:
        self.sequence_edit = QLineEdit(DEFAULT_POINT_SEQUENCE)
        self.sequence_edit.setToolTip(tr(
            "Global corner IDs separated by commas. IDs may be repeated."))
        self.repeat_spin = QSpinBox()
        self.repeat_spin.setRange(1, 20)
        self.repeat_spin.setValue(1)
        self.repeat_spin.setToolTip(tr(
            "Number of independent placements to record for every point"))
        form = QFormLayout()
        form.addRow(tr("Point sequence"), self.sequence_edit)
        form.addRow(tr("Placements per point"), self.repeat_spin)

        self.prompt_label = QLabel()
        prompt_font = self.prompt_label.font()
        prompt_font.setPointSize(prompt_font.pointSize() + 5)
        prompt_font.setBold(True)
        self.prompt_label.setFont(prompt_font)
        self.prompt_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.prompt_label.setMinimumHeight(72)

        self.live_label = QLabel(tr("Waiting for J1-J4 telemetry"))
        self.live_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.live_label.setWordWrap(True)
        self.status_label = QLabel("")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setWordWrap(True)

        self.lock_button = QPushButton(tr("Lock Joints"))
        self.lock_button.setAutoDefault(False)
        self.lock_button.setToolTip(tr(
            "Hold the current pose in closed-loop before recording"))
        self.lock_button.clicked.connect(self._request_lock)
        self.release_button = QPushButton(tr("Release Joints"))
        self.release_button.setAutoDefault(False)
        self.release_button.setToolTip(tr(
            "Release holding torque for manual placement; support the arm first"))
        self.release_button.clicked.connect(self._request_release)
        self.mode_label = QLabel()
        mode_controls = QHBoxLayout()
        mode_controls.addStretch(1)
        mode_controls.addWidget(self.lock_button)
        mode_controls.addWidget(self.release_button)
        mode_controls.addWidget(self.mode_label)
        mode_controls.addStretch(1)

        self.start_button = QPushButton(tr("Start Session (Enter)"))
        self.start_button.setAutoDefault(False)
        self.start_button.setObjectName("primaryButton")
        self.start_button.clicked.connect(self._enter_action)
        self.record_button = QPushButton(tr("Record / Next (Enter)"))
        self.record_button.setAutoDefault(False)
        self.record_button.setObjectName("primaryButton")
        self.record_button.setEnabled(False)
        self.record_button.clicked.connect(self._capture_current)
        self.undo_button = QPushButton(tr("Undo Last"))
        self.undo_button.setAutoDefault(False)
        self.undo_button.setEnabled(False)
        self.undo_button.clicked.connect(self._undo_last)
        self.finish_button = QPushButton(tr("Finish and Save"))
        self.finish_button.setAutoDefault(False)
        self.finish_button.setEnabled(False)
        self.finish_button.clicked.connect(self._finish_session)
        controls = QHBoxLayout()
        controls.addWidget(self.start_button)
        controls.addWidget(self.record_button)
        controls.addWidget(self.undo_button)
        controls.addStretch(1)
        controls.addWidget(self.finish_button)

        columns = [
            tr("No."), tr("Point"), tr("Placement"), "J1", "J2", "J3", "J4",
            "TCP X", "TCP Y", "TCP Z", tr("Pitch"), tr("Max std"),
        ]
        self.table = QTableWidget(0, len(columns))
        self.table.setHorizontalHeaderLabels(columns)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setAlternatingRowColors(True)
        # Keep calibration records readable even when Windows/Qt is using a
        # dark system palette.  A widget-local stylesheet takes precedence
        # over the inherited application palette for the viewport and headers.
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #ffffff;
                alternate-background-color: #f2f7f9;
                color: #17242c;
                gridline-color: #cbd7dc;
                border: 1px solid #b8c8cf;
                selection-background-color: #d6edf6;
                selection-color: #102a36;
            }
            QTableWidget::item {
                color: #17242c;
                padding: 4px;
            }
            QTableWidget::item:selected {
                background-color: #d6edf6;
                color: #102a36;
            }
            QHeaderView::section {
                background-color: #e5eef2;
                color: #17242c;
                border: 0;
                border-right: 1px solid #c1cfd5;
                border-bottom: 1px solid #b8c8cf;
                padding: 5px;
                font-weight: 600;
            }
            QTableCornerButton::section {
                background-color: #e5eef2;
                border: 0;
                border-right: 1px solid #c1cfd5;
                border-bottom: 1px solid #b8c8cf;
            }
        """)
        self.table.horizontalHeader().setStretchLastSection(True)

        self.path_label = QLabel("")
        self.path_label.setWordWrap(True)
        self.path_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.prompt_label)
        layout.addWidget(self.live_label)
        layout.addLayout(mode_controls)
        layout.addWidget(self.status_label)
        layout.addLayout(controls)
        layout.addWidget(self.table, 1)
        layout.addWidget(self.path_label)
        self.update_control_state()

    def _request_lock(self) -> None:
        if self._lock_joints is None:
            return
        self._lock_joints()

    def _request_release(self) -> None:
        if self._release_joints is None:
            return
        self._release_joints()

    def update_control_state(self) -> None:
        connected = (bool(self._get_connected())
                     if self._get_connected is not None else True)
        mode = (int(self._get_joints_mode())
                if self._get_joints_mode is not None else 0)
        self.lock_button.setEnabled(connected and self._lock_joints is not None)
        self.release_button.setEnabled(
            connected and self._release_joints is not None)
        self.lock_button.setProperty("modeActive", connected and mode == 2)
        self.release_button.setProperty("modeActive", connected and mode == 1)
        repolish(self.lock_button)
        repolish(self.release_button)
        if not connected:
            self.mode_label.setText(tr("Not connected"))
        elif mode == 2:
            self.mode_label.setText(tr("Locked - ready after stable"))
        elif mode == 1:
            self.mode_label.setText(tr("Released - support the arm"))
        else:
            self.mode_label.setText(tr("Joint mode unknown"))

    def observe_state(self, state: ArmState) -> None:
        """Receive a GUI telemetry snapshot; no backend calls are made here."""
        self.update_control_state()
        row = self._state_row(state)
        if row is None:
            self.live_label.setText(tr("J1-J4 are not all online"))
            return
        self._samples.append(row)
        motors = row["motor_deg"]
        speeds = row["speed_rpm"]
        self.live_label.setText(
            "J1-J4: [%s]°    %s: %.2f r/min" % (
                ", ".join("%.2f" % value for value in motors),
                tr("max speed"), max(abs(value) for value in speeds)))

    @staticmethod
    def _state_row(state: ArmState) -> Optional[dict]:
        joints = [state.joint(jid) for jid in (1, 2, 3, 4)]
        if any(joint is None or not joint.valid for joint in joints):
            return None
        motor = [float(joint.angle_deg) for joint in joints]
        speed = [float(joint.speed_rpm) for joint in joints]
        torque = [float(joint.torque_nm) for joint in joints]
        if not all(math.isfinite(value) for value in motor + speed):
            return None
        return {
            "timestamp": float(state.timestamp or time.time()),
            "source": state.source,
            "motor_deg": motor,
            "speed_rpm": speed,
            "torque_nm": torque,
        }

    def _enter_action(self) -> None:
        if self._finished:
            return
        if self._session_dir is None:
            self._start_session()
        else:
            self._capture_current()

    def _start_session(self) -> None:
        try:
            self._sequence = parse_point_sequence(self.sequence_edit.text())
        except (ValueError, TypeError) as exc:
            self._show_status(str(exc), error=True)
            return
        repeats = int(self.repeat_spin.value())
        self._steps = [
            (point_id, placement)
            for point_id in self._sequence
            for placement in range(1, repeats + 1)
        ]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(self._output_root, exist_ok=True)
        session_dir = os.path.join(self._output_root, stamp)
        suffix = 1
        while os.path.exists(session_dir):
            session_dir = os.path.join(self._output_root, "%s_%02d" % (stamp, suffix))
            suffix += 1
        os.makedirs(session_dir, exist_ok=False)
        self._session_dir = session_dir
        self._session_started_at = datetime.now(timezone.utc).astimezone().isoformat()
        self.sequence_edit.setEnabled(False)
        self.repeat_spin.setEnabled(False)
        self.start_button.setVisible(False)
        self.record_button.setEnabled(True)
        self.finish_button.setEnabled(True)
        self.path_label.setText(tr("Saving after every record to: %s") % session_dir)
        self._show_status(tr("Session started. Move to the prompted point."))
        self._save()
        self._update_prompt()

    def _recent_stable_samples(self) -> list[dict]:
        state = self._get_state()
        if state is not None:
            row = self._state_row(state)
            if (row is not None
                    and (not self._samples
                         or row["timestamp"] > self._samples[-1]["timestamp"] + 1e-9)):
                self._samples.append(row)
        if not self._samples:
            return []
        newest = self._samples[-1]["timestamp"]
        # Require a continuous stable tail.  Stable samples from the previous
        # pose must not be mixed with samples collected after a movement.
        stable_tail = []
        for row in reversed(self._samples):
            if newest - row["timestamp"] > STABLE_WINDOW_S:
                break
            if (max(abs(value) for value in row["speed_rpm"])
                    > MAX_STABLE_SPEED_RPM):
                break
            stable_tail.append(row)
        return list(reversed(stable_tail))

    def _capture_current(self) -> None:
        if self._session_dir is None or self._finished:
            return
        if self._step_index >= len(self._steps):
            self._finish_session()
            return
        if self._get_joints_mode is not None and self._get_joints_mode() != 2:
            self._show_status(tr(
                "Not recorded: lock the joints first, wait about one second, "
                "then press Enter again."), error=True)
            return
        stable = self._recent_stable_samples()
        if len(stable) < MIN_STABLE_SAMPLES:
            self._show_status(tr(
                "Not recorded: keep the arm still, wait about one second, "
                "then press Enter again (%d/%d stable samples).")
                % (len(stable), MIN_STABLE_SAMPLES), error=True)
            return

        stable = stable[-min(15, len(stable)):]
        motors = np.asarray([row["motor_deg"] for row in stable], dtype=float)
        speeds = np.asarray([row["speed_rpm"] for row in stable], dtype=float)
        torques = np.asarray([row["torque_nm"] for row in stable], dtype=float)
        motor_mean = motors.mean(axis=0)
        motor_std = motors.std(axis=0)
        speed_mean = speeds.mean(axis=0)
        torque_mean = np.nanmean(torques, axis=0)
        tcp = kinematics.fk_from_motor(motor_mean.tolist())
        point_id, placement = self._steps[self._step_index]

        target_mm = None
        level = None
        if self._target_model is not None and point_id in self._target_model.pts_mm:
            target_mm = self._target_model.pts_mm[point_id].tolist()
            level = int(self._target_model.level_of[point_id])
        record = {
            "record_index": len(self._records) + 1,
            "target_global_corner_id": point_id,
            "target_level": level,
            "placement_index": placement,
            "captured_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "sample_count": len(stable),
            "sample_window_s": float(
                stable[-1]["timestamp"] - stable[0]["timestamp"]),
            "source": stable[-1]["source"],
            "motor_mean_deg": motor_mean.tolist(),
            "motor_std_deg": motor_std.tolist(),
            "speed_mean_rpm": speed_mean.tolist(),
            "torque_mean_nm": torque_mean.tolist(),
            "tcp_base_mm": [float(value) for value in tcp[:3]],
            "tcp_pitch_deg": float(tcp[3]),
            "target_design_mm": target_mm,
        }
        self._records.append(record)
        self._step_index += 1
        self._append_table_row(record)
        self.undo_button.setEnabled(True)
        self._save()
        self._show_status(tr("Recorded point %d placement %d (%d samples).") % (
            point_id, placement, len(stable)))
        if self._step_index >= len(self._steps):
            self._finish_session()
        else:
            self._update_prompt()

    def _append_table_row(self, record: dict) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [
            record["record_index"], record["target_global_corner_id"],
            record["placement_index"], *record["motor_mean_deg"],
            *record["tcp_base_mm"], record["tcp_pitch_deg"],
            max(record["motor_std_deg"]),
        ]
        for column, value in enumerate(values):
            text = str(value) if isinstance(value, int) else "%.4f" % float(value)
            self.table.setItem(row, column, QTableWidgetItem(text))
        self.table.scrollToBottom()

    def _undo_last(self) -> None:
        if not self._records or self._finished:
            return
        removed = self._records.pop()
        self._step_index = max(0, self._step_index - 1)
        self.table.removeRow(self.table.rowCount() - 1)
        self.undo_button.setEnabled(bool(self._records))
        self._save()
        self._show_status(tr("Removed the last record for point %d.")
                          % removed["target_global_corner_id"])
        self._update_prompt()

    def _finish_session(self) -> None:
        if self._session_dir is None or self._finished:
            return
        self._finished = True
        self._save(completed=True)
        self.record_button.setEnabled(False)
        self.undo_button.setEnabled(False)
        self.finish_button.setEnabled(False)
        self.prompt_label.setText(tr("Session saved: %d records") % len(self._records))
        self._show_status(tr("Recording complete. You may close this window."))

    def _session_payload(self, completed: bool = False) -> dict:
        calibration_path = user_path("calibration", "handeye_result.yaml")
        return {
            "schema": "manual_robot_point_session_v1",
            "status": "complete" if completed or self._finished else "in_progress",
            "started_at": self._session_started_at,
            "updated_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "point_sequence": self._sequence,
            "placements_per_point": int(self.repeat_spin.value()),
            "next_step_index": self._step_index,
            "stability_gate": {
                "window_s": STABLE_WINDOW_S,
                "minimum_samples": MIN_STABLE_SAMPLES,
                "maximum_abs_speed_rpm": MAX_STABLE_SPEED_RPM,
            },
            "kinematics": {
                "module": kinematics.__name__,
                "tcp_j4_mm": kinematics.TCP_J4_MM.tolist(),
                "tcp_status": kinematics.TCP_STATUS,
            },
            "target_model": {
                "path": (self._target_model.path
                         if self._target_model is not None else None),
                "provenance": (self._target_model.provenance()
                               if self._target_model is not None else None),
            },
            "calibration_result": {
                "path": calibration_path,
                "sha256": _sha256(calibration_path),
            },
            "records": self._records,
        }

    def _save(self, completed: bool = False) -> None:
        if self._session_dir is None:
            return
        payload = self._session_payload(completed=completed)
        json_path = os.path.join(self._session_dir, "points.json")
        temp_path = json_path + ".tmp"
        with open(temp_path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temp_path, json_path)

        csv_path = os.path.join(self._session_dir, "points.csv")
        with open(csv_path, "w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "record_index", "target_global_corner_id", "target_level",
                "placement_index", "captured_at", "sample_count",
                "J1_deg", "J2_deg", "J3_deg", "J4_deg",
                "J1_std_deg", "J2_std_deg", "J3_std_deg", "J4_std_deg",
                "TCP_X_base_mm", "TCP_Y_base_mm", "TCP_Z_base_mm",
                "TCP_pitch_deg",
            ])
            for record in self._records:
                writer.writerow([
                    record["record_index"], record["target_global_corner_id"],
                    record["target_level"], record["placement_index"],
                    record["captured_at"], record["sample_count"],
                    *record["motor_mean_deg"], *record["motor_std_deg"],
                    *record["tcp_base_mm"], record["tcp_pitch_deg"],
                ])

    def _update_prompt(self) -> None:
        if self._session_dir is None:
            self.prompt_label.setText(tr(
                "Edit the sequence if needed, then press Enter to start"))
            return
        if self._step_index >= len(self._steps):
            self.prompt_label.setText(tr("All requested points are recorded"))
            return
        point_id, placement = self._steps[self._step_index]
        self.prompt_label.setText(tr(
            "Next: place the nail tip at point %d · placement %d, then press Enter")
            % (point_id, placement))

    def _show_status(self, text: str, *, error: bool = False) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #ef5350;" if error else "color: #2e7d32;")

    def closeEvent(self, event) -> None:  # noqa: N802 (Qt API)
        if self._session_dir is not None and not self._finished:
            answer = QMessageBox.question(
                self,
                tr("Finish recording?"),
                tr("The current records will be saved and the session marked complete."),
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self._finish_session()
        super().closeEvent(event)


register({
    "Manual Point Recorder": "手动点位记录器",
    "Move the nail tip to the prompted target point, wait until the arm is stationary, then press Enter. Recording never commands motion.":
        "将钉尖手动摆到提示的标定点，等待机械臂静止后按 Enter。记录器不会发送运动命令。",
    "Global corner IDs separated by commas. IDs may be repeated.":
        "用逗号分隔全局角点编号；编号可以重复。",
    "Number of independent placements to record for every point":
        "每个点需要记录的独立摆放次数",
    "Point sequence": "点位序列",
    "Placements per point": "每点摆放次数",
    "Waiting for J1-J4 telemetry": "等待 J1-J4 遥测数据",
    "J1-J4 are not all online": "J1-J4 未全部在线",
    "max speed": "最大速度",
    "Start Session (Enter)": "开始记录（Enter）",
    "Record / Next (Enter)": "记录/下一步（Enter）",
    "Undo Last": "撤销上一条",
    "Finish and Save": "结束并保存",
    "No.": "序号",
    "Point": "点位",
    "Placement": "摆放次数",
    "Pitch": "俯仰角",
    "Max std": "最大波动",
    "Enter at least one target point ID": "请至少输入一个标定点编号",
    "Target point IDs must be between 0 and 41: %s": "标定点编号必须在 0 到 41 之间：%s",
    "Saving after every record to: %s": "每条记录都会自动保存到：%s",
    "Session started. Move to the prompted point.": "记录已开始，请移动到提示点位。",
    "Not recorded: keep the arm still, wait about one second, then press Enter again (%d/%d stable samples).":
        "未记录：请保持机械臂静止约一秒，再按一次 Enter（稳定样本 %d/%d）。",
    "Not recorded: lock the joints first, wait about one second, then press Enter again.":
        "未记录：请先锁定关节，保持静止约一秒，然后再按一次 Enter。",
    "Recorded point %d placement %d (%d samples).": "已记录点 %d，第 %d 次摆放（%d 个样本）。",
    "Removed the last record for point %d.": "已撤销点 %d 的上一条记录。",
    "Session saved: %d records": "记录完成：共 %d 条",
    "Recording complete. You may close this window.": "记录已保存，可以关闭窗口。",
    "Edit the sequence if needed, then press Enter to start": "如需可修改点位序列，然后按 Enter 开始",
    "All requested points are recorded": "所有指定点位均已记录",
    "Next: place the nail tip at point %d · placement %d, then press Enter":
        "下一步：将钉尖摆到 %d 号点（第 %d 次），然后按 Enter",
    "Finish recording?": "结束记录？",
    "The current records will be saved and the session marked complete.":
        "当前记录将被保存，并把本次会话标记为完成。",
})

register({
    "Hold the current pose in closed-loop before recording":
        "进入闭环并保持当前位置；记录前必须锁定关节",
    "Release holding torque for manual placement; support the arm first":
        "释放保持力矩以便手动摆放；请先托住机械臂",
    "Locked - ready after stable": "已锁定——静止后可记录",
    "Released - support the arm": "已释放——请托住机械臂",
    "Joint mode unknown": "关节状态未知",
})
