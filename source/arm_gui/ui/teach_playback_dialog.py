# -*- coding: utf-8 -*-
"""Teach-and-playback dialog for manual drag demonstrations.

Workflow:
1) Release joints and manually drag the arm (and gripper if needed).
2) Start recording, perform the motion, then stop recording.
3) Lock joints and playback the captured trajectory once or in loops.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time
from datetime import datetime, timezone
from typing import Callable, Optional, Sequence

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from ..core.apppaths import user_path
from ..core.state import ArmState
from ..i18n import register, tr
from .style import FG_TEXT, RED, YELLOW

# 与遥测日志同一套配色，回放状态行的等级一眼可辨
_LEVEL_COLOR = {'info': FG_TEXT, 'warn': YELLOW, 'error': RED}
_RECORDING_SCHEMA = 'daran_teach_recording'
_RECORDING_VERSION = 1


class TeachPlaybackDialog(QDialog):
    """Non-modal teach dialog: record from live state, playback via signal."""

    playback_requested = Signal(object, int, float, float)  # frames, loops, speed, accel
    playback_stop_requested = Signal()

    def __init__(
        self,
        *,
        get_connected: Callable[[], bool],
        get_joints_mode: Callable[[], int],
        lock_joints: Callable[[], None],
        release_joints: Callable[[], None],
        joint_ids: Sequence[int],
        default_speed: float,
        default_accel: float,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle(tr('Teach Playback'))
        self.resize(760, 250)
        self.setModal(False)

        self._get_connected = get_connected
        self._get_joints_mode = get_joints_mode
        self._lock_joints = lock_joints
        self._release_joints = release_joints
        self._joint_ids = tuple(int(jid) for jid in joint_ids)

        self._recording = False
        self._release_pending = False
        self._record_started_at = 0.0
        self._frames: list[dict] = []
        self._playing = False
        self._shutdown_prepared = False
        self._recordings_dir = user_path('teach_recordings')
        self._recording_path: Optional[str] = None
        self._recording_created_at: Optional[str] = None

        self._build_ui(default_speed, default_accel)
        self.update_control_state()

    def _build_ui(self, default_speed: float, default_accel: float) -> None:
        self.mode_label = QLabel('')
        self.mode_label.setWordWrap(True)

        self.stats_label = QLabel(tr('No recording yet'))
        self.stats_label.setWordWrap(True)

        self.btn_lock = QPushButton(tr('Lock Joints'))
        self.btn_lock.clicked.connect(self._lock_joints)
        self.btn_release = QPushButton(tr('Release Joints'))
        self.btn_release.clicked.connect(self._release_joints)

        mode_bar = QHBoxLayout()
        mode_bar.addWidget(self.btn_lock)
        mode_bar.addWidget(self.btn_release)
        mode_bar.addWidget(self.mode_label, 1)

        self.btn_start = QPushButton(tr('Start Recording'))
        self.btn_start.setObjectName('primaryButton')
        self.btn_start.clicked.connect(self._start_recording)

        self.btn_stop = QPushButton(tr('Stop Recording'))
        self.btn_stop.clicked.connect(self._stop_recording)

        self.btn_clear = QPushButton(tr('Clear Recording'))
        self.btn_clear.clicked.connect(self._clear_recording)

        self.btn_open_file = QPushButton(tr('Open Recording…'))
        self.btn_open_file.clicked.connect(self._open_recording)
        self.btn_save_as = QPushButton(tr('Save As…'))
        self.btn_save_as.clicked.connect(self._save_recording_as)

        rec_bar = QHBoxLayout()
        rec_bar.addWidget(self.btn_start)
        rec_bar.addWidget(self.btn_stop)
        rec_bar.addWidget(self.btn_clear)
        rec_bar.addStretch(1)
        rec_bar.addWidget(self.btn_open_file)
        rec_bar.addWidget(self.btn_save_as)

        self.file_label = QLabel(tr('No recording file selected'))
        self.file_label.setWordWrap(True)
        self.file_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)

        self.loop_spin = QSpinBox()
        self.loop_spin.setRange(1, 100)
        self.loop_spin.setValue(1)
        self.loop_spin.setToolTip(tr('Number of playback loops'))

        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.5, 100.0)
        self.speed_spin.setDecimals(1)
        self.speed_spin.setValue(float(default_speed))
        self.speed_spin.setSuffix(' r/min')

        self.accel_spin = QDoubleSpinBox()
        self.accel_spin.setRange(1.0, 200.0)
        self.accel_spin.setDecimals(1)
        self.accel_spin.setValue(float(default_accel))

        self.btn_play = QPushButton(tr('Play Recording'))
        self.btn_play.setObjectName('primaryButton')
        self.btn_play.clicked.connect(self._request_playback)

        self.btn_stop_play = QPushButton(tr('Stop Playback'))
        self.btn_stop_play.setEnabled(False)
        self.btn_stop_play.clicked.connect(self.playback_stop_requested)

        # 回放状态回显：主窗口把排队/等待/开始/失败原因同步到这里，
        # 避免用户只能去右侧日志里找为什么点了没动
        self.playback_label = QLabel('')
        self.playback_label.setWordWrap(True)

        play_bar = QHBoxLayout()
        play_bar.addWidget(QLabel(tr('Loops')))
        play_bar.addWidget(self.loop_spin)
        play_bar.addSpacing(8)
        play_bar.addWidget(QLabel(tr('Speed')))
        play_bar.addWidget(self.speed_spin)
        play_bar.addSpacing(8)
        play_bar.addWidget(QLabel(tr('Acceleration')))
        play_bar.addWidget(self.accel_spin)
        play_bar.addStretch(1)
        play_bar.addWidget(self.btn_play)
        play_bar.addWidget(self.btn_stop_play)

        layout = QVBoxLayout(self)
        layout.addLayout(mode_bar)
        layout.addLayout(rec_bar)
        layout.addWidget(self.stats_label)
        layout.addWidget(self.file_label)
        layout.addLayout(play_bar)
        layout.addWidget(self.playback_label)

    def update_control_state(self) -> None:
        connected = bool(self._get_connected())
        mode = int(self._get_joints_mode()) if connected else 0
        mode_text = tr('Disconnected')
        if connected:
            if mode == 2:
                mode_text = tr('Locked')
            elif mode == 1:
                mode_text = tr('Released')
            else:
                mode_text = tr('Switching / unknown')

        # set_joints_mode() is asynchronous.  Sampling must not begin until
        # the backend has read back and emitted the released state.
        if self._release_pending and connected and mode == 1:
            self._release_pending = False
            self._recording = True
            self._record_started_at = 0.0
            self.set_playback_status(tr('Joints released; recording started'))
            self._refresh_stats()
        elif self._release_pending and not connected:
            self._release_pending = False
            self.set_playback_status(
                tr('Recording cancelled: connection lost before release'), 'warn')
        self.mode_label.setText(tr('Mode: %s') % mode_text)

        # Playback includes the main window's pending lock/enable and
        # preposition phases, not just dense tracking.  Recording/release
        # switching also owns the mode controls until it reaches a stable state.
        mode_controls_allowed = (
            connected and not self._playing and not self._recording
            and not self._release_pending)
        self.btn_lock.setEnabled(mode_controls_allowed)
        self.btn_release.setEnabled(mode_controls_allowed)
        self.btn_start.setEnabled(connected and not self._recording
                                  and not self._release_pending
                                  and not self._playing)
        self.btn_stop.setEnabled(
            connected and (self._recording or self._release_pending))
        self.btn_clear.setEnabled(not self._playing)
        file_controls_allowed = (
            not self._playing and not self._recording
            and not self._release_pending)
        self.btn_open_file.setEnabled(file_controls_allowed)
        self.btn_save_as.setEnabled(file_controls_allowed and bool(self._frames))
        # 有连接就允许点击：样本不足/正在录制等原因由 _request_playback 明确回显，
        # 否则按钮变灰、点了毫无反应，用户无从判断卡在哪一步
        self.btn_play.setEnabled(connected and not self._playing
                                 and not self._recording
                                 and not self._release_pending)
        self.btn_stop_play.setEnabled(self._playing)

    @property
    def is_recording(self) -> bool:
        return self._recording or self._release_pending

    def set_playback_status(self, text: str, level: str = 'info') -> None:
        """Echo the main window's playback feedback inside the dialog."""
        self.playback_label.setStyleSheet(
            'color: %s;' % _LEVEL_COLOR.get(level, FG_TEXT))
        self.playback_label.setText(text)

    def _set_file_status(self, text: str, level: str = 'info') -> None:
        self.file_label.setStyleSheet(
            'color: %s;' % _LEVEL_COLOR.get(level, FG_TEXT))
        self.file_label.setText(text)

    def set_playback_active(self, active: bool) -> None:
        """Playback pending/running: block manual controls and offer Stop."""
        active = bool(active)
        if active == self._playing:
            return
        self._playing = active
        self.update_control_state()

    def prepare_for_shutdown(self, final_close: bool = False) -> bool:
        """Quiesce UI state and tell the owner whether a lock is required.

        The main window submits the actual hold/lock requests before backend
        teardown.  If recording contains frames, this method also performs a
        best-effort atomic save; failures are reported but never clear memory
        or prevent the caller from requesting the safety lock.
        """
        need_lock = (self.is_recording
                     or (self._get_connected()
                         and int(self._get_joints_mode()) == 1))
        if self.is_recording and self._frames:
            # Best effort only: a disk error must never prevent the main window
            # from submitting the safety lock below.
            self._auto_save_recording()
        self._recording = False
        self._release_pending = False
        self._playing = False
        # Suppress the child closeEvent only for application shutdown.  A
        # backend disconnect/switch leaves this dialog reusable after reconnect.
        self._shutdown_prepared = bool(final_close)
        self.update_control_state()
        return need_lock

    def observe_state(self, state: ArmState) -> None:
        motors = []
        valid = True
        for jid in self._joint_ids:
            reading = state.joint(jid)
            if reading is None or not reading.valid:
                valid = False
                break
            motors.append(float(reading.angle_deg))

        gripper = None
        if state.gripper is not None and state.gripper.valid:
            gripper = float(state.gripper.angle_deg)

        if valid:
            valid = all(math.isfinite(angle) for angle in motors)
        if gripper is not None and not math.isfinite(gripper):
            gripper = None

        if self._recording and valid:
            now = float(state.timestamp) if state.timestamp else time.time()
            if self._record_started_at <= 0.0:
                self._record_started_at = now
            t_rel = max(0.0, now - self._record_started_at)
            # Polling is typically <=20-50 Hz; keep a lightweight timestamp guard.
            if self._frames and t_rel <= self._frames[-1]['t'] + 1e-6:
                t_rel = self._frames[-1]['t'] + 0.001
            self._frames.append({
                't': t_rel,
                'joints': dict(zip(self._joint_ids, motors)),
                'gripper': gripper,
            })
            self._refresh_stats()

    def _start_recording(self) -> None:
        if not self._get_connected():
            return
        if self._recording:
            return
        ret = QMessageBox.question(
            self,
            tr('Start Teach Recording'),
            tr('Starting teach recording will RELEASE the joints immediately.\n\n'
               'The arm may sag under gravity. Please support the arm first and '
               'keep people/obstacles clear.\n\nProceed to release and record?'),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if ret != QMessageBox.StandardButton.Yes:
            return
        # Mode switching is asynchronous: request release now, but begin
        # appending samples only after update_control_state observes mode=1.
        self._recording = False
        self._release_pending = True
        self._record_started_at = 0.0
        self._frames = []
        self._recording_path = None
        self._recording_created_at = self._now_iso()
        self._set_file_status(tr('Recording has not been saved yet'))
        self.set_playback_status(tr('Waiting for joints to enter released mode…'))
        self._release_joints()
        self._refresh_stats()
        self.update_control_state()

    def _stop_recording(self) -> None:
        if not self._recording and not self._release_pending:
            return
        self._recording = False
        self._release_pending = False
        # Stop recording means lock joints back for safe playback.
        self._lock_joints()
        # Lock is intentionally requested first.  Persistence is best effort
        # and never clears the in-memory frames on failure.
        if self._frames:
            self._auto_save_recording()
        else:
            self._set_file_status(
                tr('Recording stopped without samples; no file was created'),
                'warn')
        self._refresh_stats()
        self.update_control_state()

    def _clear_recording(self) -> None:
        was_active = self._recording or self._release_pending
        self._recording = False
        self._release_pending = False
        self._record_started_at = 0.0
        self._frames = []
        self._recording_path = None
        self._recording_created_at = None
        self._set_file_status(tr('No recording file selected'))
        if was_active:
            self._lock_joints()
        self._refresh_stats()
        self.update_control_state()

    @staticmethod
    def _now_iso() -> str:
        return datetime.now(timezone.utc).astimezone().isoformat(
            timespec='milliseconds')

    @staticmethod
    def _finite_number(value) -> bool:
        return (isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value)))

    def _recording_payload(self) -> dict:
        if self._recording_created_at is None:
            self._recording_created_at = self._now_iso()
        frames = []
        for frame in self._frames:
            raw_joints = frame.get('joints') or {}
            joints = {
                str(jid): float(raw_joints[jid])
                for jid in self._joint_ids
            }
            gripper = frame.get('gripper')
            frames.append({
                't': float(frame.get('t', 0.0)),
                'joints': joints,
                'gripper': None if gripper is None else float(gripper),
            })
        return {
            'schema': _RECORDING_SCHEMA,
            'version': _RECORDING_VERSION,
            'created_at': self._recording_created_at,
            'joint_ids': list(self._joint_ids),
            'frames': frames,
        }

    def _validate_recording_payload(
            self, payload: object) -> tuple[list[dict], str]:
        """Validate an entire JSON document without mutating dialog state."""
        if not isinstance(payload, dict):
            raise ValueError(tr('Recording JSON root must be an object'))
        if payload.get('schema') != _RECORDING_SCHEMA:
            raise ValueError(tr('Unsupported recording schema: %s')
                             % payload.get('schema'))
        version = payload.get('version')
        if (not isinstance(version, int) or isinstance(version, bool)
                or version != _RECORDING_VERSION):
            raise ValueError(tr('Unsupported recording version: %s') % version)

        created_at = payload.get('created_at')
        if not isinstance(created_at, str) or not created_at.strip():
            raise ValueError(tr('Recording created_at must be an ISO-8601 string'))
        try:
            parsed_created_at = datetime.fromisoformat(
                created_at.strip().replace('Z', '+00:00'))
        except ValueError as exc:
            raise ValueError(
                tr('Recording created_at must be an ISO-8601 string')) from exc
        if (parsed_created_at.tzinfo is None
                or parsed_created_at.utcoffset() is None):
            raise ValueError(tr('Recording created_at must include a timezone'))
        created_at = created_at.strip()

        joint_ids = payload.get('joint_ids')
        if (not isinstance(joint_ids, list)
                or any(not isinstance(jid, int) or isinstance(jid, bool)
                       for jid in joint_ids)):
            raise ValueError(tr('Recording joint_ids must be an integer list'))
        if tuple(joint_ids) != self._joint_ids:
            raise ValueError(
                tr('Recording joint IDs %s do not match configured joints %s')
                % (joint_ids, list(self._joint_ids)))

        raw_frames = payload.get('frames')
        if not isinstance(raw_frames, list):
            raise ValueError(tr('Recording frames must be a list'))
        normalized: list[dict] = []
        previous_t = -1.0
        expected_keys = {str(jid) for jid in self._joint_ids}
        for index, frame in enumerate(raw_frames):
            if not isinstance(frame, dict):
                raise ValueError(tr('Recording frame %d must be an object') % index)
            timestamp = frame.get('t')
            if (not self._finite_number(timestamp)
                    or float(timestamp) < 0.0):
                raise ValueError(
                    tr('Recording frame %d has an invalid timestamp') % index)
            timestamp = float(timestamp)
            if index > 0 and timestamp <= previous_t:
                raise ValueError(tr(
                    'Recording timestamps must be strictly increasing '
                    '(frame %d)') % index)
            previous_t = timestamp

            raw_joints = frame.get('joints')
            if not isinstance(raw_joints, dict):
                raise ValueError(
                    tr('Recording frame %d joints must be an object') % index)
            if set(raw_joints) != expected_keys:
                raise ValueError(tr(
                    'Recording frame %d must contain exactly joint IDs %s')
                    % (index, list(self._joint_ids)))
            joints = {}
            for jid in self._joint_ids:
                value = raw_joints[str(jid)]
                if not self._finite_number(value):
                    raise ValueError(tr(
                        'Recording frame %d J%d angle must be finite')
                        % (index, jid))
                joints[jid] = float(value)

            if 'gripper' not in frame:
                raise ValueError(tr(
                    'Recording frame %d is missing gripper data') % index)
            gripper = frame.get('gripper')
            if gripper is not None and not self._finite_number(gripper):
                raise ValueError(tr(
                    'Recording frame %d gripper angle must be finite or null')
                    % index)
            normalized.append({
                't': timestamp,
                'joints': joints,
                'gripper': None if gripper is None else float(gripper),
            })
        return normalized, created_at

    def _write_recording_file(self, path: str) -> str:
        """Atomically persist current frames and return the absolute path."""
        absolute = os.path.abspath(path)
        directory = os.path.dirname(absolute)
        if not directory:
            raise ValueError(tr('Recording file has no parent directory'))
        os.makedirs(directory, exist_ok=True)
        payload = self._recording_payload()
        # Refuse to write a malformed in-memory document as a supposedly valid
        # versioned file.
        self._validate_recording_payload(payload)
        fd, temporary = tempfile.mkstemp(
            prefix='.teach_recording_', suffix='.tmp', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2,
                          allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, absolute)
        finally:
            if os.path.exists(temporary):
                try:
                    os.unlink(temporary)
                except OSError:
                    pass
        return absolute

    def _next_auto_recording_path(self) -> str:
        stamp = datetime.now().astimezone().strftime(
            '%Y%m%d_%H%M%S_%f')[:-3]
        stem = 'teach_recording_%s' % stamp
        candidate = os.path.join(self._recordings_dir, stem + '.json')
        suffix = 1
        while os.path.exists(candidate):
            candidate = os.path.join(
                self._recordings_dir, '%s_%02d.json' % (stem, suffix))
            suffix += 1
        return candidate

    def _auto_save_recording(self) -> Optional[str]:
        if not self._frames:
            return None
        path = self._recordings_dir
        try:
            path = self._next_auto_recording_path()
            saved = self._write_recording_file(path)
        except Exception as exc:
            message = tr('Failed to save recording "%s": %s') % (path, exc)
            self._set_file_status(message, 'error')
            self.set_playback_status(message, 'error')
            return None
        self._recording_path = saved
        self._set_file_status(tr('Recording file: %s') % saved)
        self.set_playback_status(
            tr('Recording saved automatically: %s') % saved)
        return saved

    def _open_recording(self) -> None:
        if self._playing or self.is_recording:
            self.set_playback_status(
                tr('Stop the current teach operation before opening a file'),
                'warn')
            return
        path, _selected_filter = QFileDialog.getOpenFileName(
            self, tr('Open Teach Recording'), self._recordings_dir,
            tr('Teach recording JSON (*.json);;All files (*)'))
        if not path:
            return
        absolute = os.path.abspath(path)
        try:
            with open(absolute, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
            # Validate into locals first.  A failure below must preserve both
            # the old trajectory and its displayed path.
            frames, created_at = self._validate_recording_payload(payload)
        except (OSError, UnicodeError, json.JSONDecodeError,
                TypeError, ValueError) as exc:
            message = tr('Failed to load recording "%s": %s') % (absolute, exc)
            self._set_file_status(message, 'error')
            self.set_playback_status(message, 'error')
            return

        self._frames = frames
        self._record_started_at = 0.0
        self._recording_created_at = created_at
        self._recording_path = absolute
        self._refresh_stats()
        self._set_file_status(tr('Recording file: %s') % absolute)
        self.set_playback_status(tr('Recording loaded: %s') % absolute)
        self.update_control_state()

    def _save_recording_as(self) -> None:
        if self._playing or self.is_recording:
            self.set_playback_status(
                tr('Stop the current teach operation before saving a file'),
                'warn')
            return
        if not self._frames:
            self.set_playback_status(tr('No recording samples to save'), 'warn')
            return
        default_name = (os.path.basename(self._recording_path)
                        if self._recording_path
                        else os.path.basename(self._next_auto_recording_path()))
        suggested = os.path.join(self._recordings_dir, default_name)
        path, _selected_filter = QFileDialog.getSaveFileName(
            self, tr('Save Teach Recording As'), suggested,
            tr('Teach recording JSON (*.json);;All files (*)'))
        if not path:
            return
        if not os.path.splitext(path)[1]:
            path += '.json'
        try:
            saved = self._write_recording_file(path)
        except Exception as exc:
            message = tr('Failed to save recording "%s": %s') % (path, exc)
            self._set_file_status(message, 'error')
            self.set_playback_status(message, 'error')
            return
        self._recording_path = saved
        self._set_file_status(tr('Recording file: %s') % saved)
        self.set_playback_status(tr('Recording saved as: %s') % saved)
        self.update_control_state()

    def _request_playback(self) -> None:
        if self.is_recording:
            self.set_playback_status(
                tr('Stop recording before playing back'), 'warn')
            return
        if len(self._frames) < 2:
            self.set_playback_status(tr('No recording yet'), 'warn')
            return
        if not self._get_connected():
            self.set_playback_status(tr('Disconnected'), 'warn')
            return
        self.set_playback_status(tr('Playback requested…'))
        self.playback_requested.emit(
            list(self._frames),
            int(self.loop_spin.value()),
            float(self.speed_spin.value()),
            float(self.accel_spin.value()),
        )

    def _refresh_stats(self) -> None:
        if not self._frames:
            if self._release_pending:
                status = tr('Waiting for released mode...')
            else:
                status = tr('Recording...') if self._recording else tr('No recording yet')
            self.stats_label.setText(status)
            return
        duration = float(self._frames[-1]['t'])
        self.stats_label.setText(
            tr('Samples: %d | Duration: %.2f s') % (len(self._frames), duration))

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._shutdown_prepared:
            super().closeEvent(event)
            return
        was_recording = self.is_recording
        need_lock = (was_recording
                     or (self._get_connected()
                         and int(self._get_joints_mode()) == 1))
        self._recording = False
        self._release_pending = False
        # Always cancel: playback may still be queued waiting for lock/enable.
        # MainWindow treats that pending phase as active and will request a hold
        # whenever a trajectory token already exists.
        self._playing = False
        self.playback_stop_requested.emit()
        if need_lock:
            self._lock_joints()
        if was_recording and self._frames:
            # Safety request first; persistence is best effort and cannot erase
            # or otherwise alter the in-memory trajectory on failure.
            self._auto_save_recording()
        super().closeEvent(event)


register({
    'Teach Playback': '示教回放',
    'No recording yet': '暂无录制数据',
    'Lock Joints': '锁定关节',
    'Release Joints': '释放关节',
    'Start Recording': '开始录制',
    'Start Teach Recording': '开始示教录制',
    'Stop Recording': '停止录制',
    'Starting teach recording will RELEASE the joints immediately.\n\n'
    'The arm may sag under gravity. Please support the arm first and '
    'keep people/obstacles clear.\n\nProceed to release and record?':
        '开始示教录制将立即【释放关节】。\n\n机械臂可能在重力作用下下垂，请先扶稳机械臂，'
        '并确认周围无人员和障碍物。\n\n是否继续释放并录制？',
    'Clear Recording': '清空录制',
    'Open Recording…': '打开录制文件…',
    'Save As…': '另存为…',
    'No recording file selected': '尚未选择录制文件',
    'Recording has not been saved yet': '当前录制尚未保存',
    'Recording stopped without samples; no file was created':
        '录制已停止，但没有样本，因此未创建文件',
    'Recording file: %s': '录制文件：%s',
    'Recording saved automatically: %s': '录制已自动保存：%s',
    'Recording saved as: %s': '录制已另存为：%s',
    'Recording loaded: %s': '已加载录制：%s',
    'Failed to save recording "%s": %s': '保存录制“%s”失败：%s',
    'Failed to load recording "%s": %s': '加载录制“%s”失败：%s',
    'Open Teach Recording': '打开示教录制文件',
    'Save Teach Recording As': '示教录制另存为',
    'Teach recording JSON (*.json);;All files (*)':
        '示教录制 JSON (*.json);;所有文件 (*)',
    'Stop the current teach operation before opening a file':
        '请先停止当前示教操作，再打开文件',
    'Stop the current teach operation before saving a file':
        '请先停止当前示教操作，再保存文件',
    'No recording samples to save': '没有可保存的录制样本',
    'Recording file has no parent directory': '录制文件没有有效的上级目录',
    'Recording JSON root must be an object': '录制 JSON 根节点必须是对象',
    'Unsupported recording schema: %s': '不支持的录制 schema：%s',
    'Unsupported recording version: %s': '不支持的录制版本：%s',
    'Recording created_at must be an ISO-8601 string':
        '录制 created_at 必须是 ISO-8601 字符串',
    'Recording created_at must include a timezone':
        '录制 created_at 必须包含时区',
    'Recording joint_ids must be an integer list':
        '录制 joint_ids 必须是整数列表',
    'Recording joint IDs %s do not match configured joints %s':
        '录制关节 ID %s 与当前配置关节 %s 不一致',
    'Recording frames must be a list': '录制 frames 必须是列表',
    'Recording frame %d must be an object': '录制第 %d 帧必须是对象',
    'Recording frame %d has an invalid timestamp':
        '录制第 %d 帧时间戳无效',
    'Recording timestamps must be strictly increasing (frame %d)':
        '录制时间戳必须严格递增（第 %d 帧）',
    'Recording frame %d joints must be an object':
        '录制第 %d 帧 joints 必须是对象',
    'Recording frame %d must contain exactly joint IDs %s':
        '录制第 %d 帧必须且只能包含关节 ID %s',
    'Recording frame %d J%d angle must be finite':
        '录制第 %d 帧 J%d 角度必须是有限数值',
    'Recording frame %d is missing gripper data':
        '录制第 %d 帧缺少 gripper 数据',
    'Recording frame %d gripper angle must be finite or null':
        '录制第 %d 帧夹爪角度必须是有限数值或 null',
    'Number of playback loops': '回放循环次数',
    'Play Recording': '回放录制',
    'Stop Playback': '停止回放',
    'Playback requested…': '已请求回放…',
    'Stop recording before playing back': '请先停止录制再回放',
    'Loops': '循环',
    'Speed': '速度',
    'Acceleration': '加速度',
    'Mode: %s': '模式：%s',
    'Disconnected': '未连接',
    'Locked': '已锁定',
    'Released': '已释放',
    'Switching / unknown': '切换中/未知',
    'Waiting for joints to enter released mode…': '正在等待关节进入释放模式…',
    'Joints released; recording started': '关节已释放，示教录制已开始',
    'Recording cancelled: connection lost before release': '关节释放前连接已中断，录制已取消',
    'Waiting for released mode...': '等待关节释放...',
    'Recording...': '录制中...',
    'Samples: %d | Duration: %.2f s': '样本数：%d | 时长：%.2f 秒',
})
