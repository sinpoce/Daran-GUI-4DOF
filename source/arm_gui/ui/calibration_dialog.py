# -*- coding: utf-8 -*-
"""GUI workflow for the official RGB-D + IMU multi-plane calibration."""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import asdict
from datetime import datetime
from typing import Callable, Dict, List, Optional

import numpy as np
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QDialog, QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit, QPushButton,
    QSizePolicy, QVBoxLayout, QWidget,
)

from ..core import calibration, kinematics, paper_archive, target3d
from ..i18n import tr
from .style import GREEN, RED, mono_font
from .widgets import confirm

_SPEED_RPM = 5.0
_ACCEL = 10.0
_TICK_MS = 120
_STATE_MAX_AGE_S = 2.0
_AT_HOME_TOL_DEG = 1.0
_ANGLE_TOL_DEG = 0.45
_SPEED_TOL_RPM = 0.6
_STABLE_TICKS = 3
_SETTLE_TIMEOUT_S = 10.0
_CAPTURE_SECONDS = 1.35
_MIN_GRAY_FRAMES = 5
_MIN_DEPTH_FRAMES = 8
_MIN_ACCEL_ROWS = 12

_TEST_POINT_IDS = (0, 6, 7, 13, 14, 20, 21, 27, 28, 34, 35, 41)
_TEST_CLEARANCE_MM = 30.0
_TEST_TRANSIT_MM = 150.0
_TEST_CARTESIAN_STEP_MM = 15.0
_TEST_MAX_MOTOR_STEP_DEG = 4.0
_TEST_SPEED_RPM = 3.0
_TEST_ACCEL = 5.0


class _StreamWorker(QThread):
    ready = Signal(object)
    packet = Signal(object)
    failed = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop_requested = False

    def stop(self) -> None:
        self._stop_requested = True

    def run(self) -> None:
        camera = calibration.CalibrationCamera()
        try:
            self.ready.emit(camera.open())
            while not self._stop_requested:
                try:
                    packet = camera.grab_packet(timeout_ms=800)
                except Exception:
                    if self._stop_requested:
                        break
                    self.msleep(25)
                    continue
                if packet is not None:
                    self.packet.emit(packet)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            camera.close()


class CalibrationDialog(QDialog):
    """Non-modal, safety-gated formal calibration and point-test dialog."""

    def __init__(self, cfg, get_backend: Callable[[], object],
                 get_state: Callable[[], object], camera_panel,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._get_backend = get_backend
        self._get_state = get_state
        self._camera_panel = camera_panel
        self._home = self._load_home()
        self._home_set = len(self._home) == 4
        self._model = target3d.TargetModel(target3d.default_model_path())
        self._worker: Optional[_StreamWorker] = None
        self._camera_ready = False
        self._camera_error = ''
        self._K = self._dist = self._R_color_imu = None
        self._camera_info: Dict = {}
        self._last_packet_t = 0.0
        self._latest_gray = self._latest_depth = None
        self._live_corners: Dict[int, np.ndarray] = {}
        self._last_detection_t = 0.0
        self._last_overlay = None
        self._gray_buffer = deque(maxlen=12)
        self._depth_buffer = deque(maxlen=16)
        self._accel_buffer = deque(maxlen=240)
        self._packet_buffer = deque(maxlen=96)
        self._capture_started = 0.0
        self._running = False
        self._phase = 'idle'
        self._index = 0
        self._targets: List[Dict[int, float]] = []
        self._target: Dict[int, float] = {}
        self._settle_started = 0.0
        self._stable_ticks = 0
        self._samples: List[Dict] = []
        self._quality_rows: List[Dict] = []
        self._result: Optional[Dict] = None
        self._last_saved_path = ''
        self._formal_promoted = False
        self._archive: Optional[paper_archive.PaperCalibrationArchive] = None

        self._test_running = False
        self._test_waiting_continue = False
        self._test_point_index = -1
        self._test_plans: List[List[Dict[int, float]]] = []
        self._test_queue: List[Dict[int, float]] = []
        self._test_target: Dict[int, float] = {}
        self._test_settle_started = 0.0
        self._test_stable_ticks = 0
        self._test_board: Optional[np.ndarray] = None

        self.setWindowTitle('3D Hand-Eye Calibration')
        self.setModal(False)
        self.setMinimumSize(650, 760)
        self._timer = QTimer(self)
        self._timer.setInterval(_TICK_MS)
        self._timer.timeout.connect(self._tick)
        self._build_ui()
        self._update_buttons()
        self._update_progress()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        status_row = QHBoxLayout()
        self.btn_home = QPushButton(tr('Calibration Point'))
        self.btn_home.clicked.connect(self._move_home)
        self.lbl_target = QLabel('Target: opening camera...')
        self.lbl_target.setFont(mono_font(10, bold=True))
        status_row.addWidget(self.btn_home)
        status_row.addWidget(self.lbl_target, 1)
        layout.addLayout(status_row)

        button_row = QHBoxLayout()
        self.btn_start = QPushButton('Start Calibration')
        self.btn_start.setObjectName('primaryButton')
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop = QPushButton(tr('Stop'))
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_save = QPushButton(tr('Save Result'))
        self.btn_save.clicked.connect(self._on_save_result)
        self.btn_test = QPushButton('Test Points')
        self.btn_test.clicked.connect(self._on_test_start)
        self.btn_continue = QPushButton('Continue')
        self.btn_continue.clicked.connect(self._on_test_continue)
        for button in (self.btn_start, self.btn_stop, self.btn_save,
                       self.btn_test, self.btn_continue):
            button_row.addWidget(button)
        button_row.addStretch(1)
        layout.addLayout(button_row)

        progress_row = QHBoxLayout()
        self.lbl_step = QLabel('')
        self.lbl_phase = QLabel('')
        self.lbl_phase.setObjectName('dimLabel')
        self.lbl_count = QLabel('')
        progress_row.addWidget(self.lbl_step)
        progress_row.addWidget(self.lbl_phase)
        progress_row.addStretch(1)
        progress_row.addWidget(self.lbl_count)
        layout.addLayout(progress_row)
        self.lbl_detail = QLabel('')
        self.lbl_detail.setWordWrap(True)
        self.lbl_detail.setObjectName('dimLabel')
        layout.addWidget(self.lbl_detail)

        self.overlay = QLabel('Opening RGB-D/IMU camera...')
        self.overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.overlay.setObjectName('imagePlaceholder')
        self.overlay.setMinimumSize(560, 350)
        self.overlay.setSizePolicy(QSizePolicy.Policy.Expanding,
                                   QSizePolicy.Policy.Expanding)
        layout.addWidget(self.overlay, 1)
        layout.addWidget(QLabel(tr('Result')))
        self.txt_result = QPlainTextEdit()
        self.txt_result.setReadOnly(True)
        self.txt_result.setFont(mono_font(9))
        self.txt_result.setMinimumHeight(170)
        layout.addWidget(self.txt_result)

    def showEvent(self, event) -> None:  # noqa: N802
        super().showEvent(event)
        if self._worker is None:
            try:
                self._camera_panel.stop()
            except Exception:
                pass
            self._worker = _StreamWorker(self)
            self._worker.ready.connect(self._on_camera_ready)
            self._worker.packet.connect(self._on_packet)
            self._worker.failed.connect(self._on_camera_failed)
            self._worker.start()

    def _on_camera_ready(self, info: Dict) -> None:
        self._camera_info = dict(info)
        self._K = np.asarray(info['K'], float)
        self._dist = np.asarray(info['dist'], float)
        self._R_color_imu = np.asarray(info['R_color_imu'], float)
        self._camera_ready = True
        self._camera_error = ''
        self.lbl_detail.setText(
            'Camera ready. Keep all three target strips visible and move to the '
            'saved calibration point.')
        self._update_buttons()

    def _on_camera_failed(self, message: str) -> None:
        self._camera_ready = False
        self._camera_error = message
        self.overlay.setText('RGB-D/IMU camera failed: %s' % message)
        self.lbl_target.setText('Target: camera unavailable')
        self.lbl_target.setStyleSheet('color: %s;' % RED)
        self._update_buttons()

    def _on_packet(self, packet: Dict) -> None:
        stamp = float(packet.get('timestamp', time.monotonic()))
        self._last_packet_t = stamp
        self._latest_gray = packet['gray']
        self._latest_depth = packet['depth_m']
        self._gray_buffer.append((stamp, packet['gray']))
        self._depth_buffer.append((stamp, packet['depth_m']))
        self._packet_buffer.append((stamp, packet))
        for row in packet.get('accel', []):
            self._accel_buffer.append((stamp, row))
        now = time.monotonic()
        if now - self._last_detection_t < 0.14:
            return
        self._last_detection_t = now
        try:
            self._live_corners = target3d.detect_corners(packet['gray'])
            self._set_overlay(calibration.draw_overlay(
                packet['gray'], self._live_corners))
        except Exception as exc:
            self.lbl_detail.setText('Target detection error: %s' % exc)
            return
        levels = sorted({self._model.level_of[g] for g in self._live_corners})
        passed = (len(self._live_corners) >=
                  calibration.MIN_PREFLIGHT_CORNERS and len(levels) == 3)
        self.lbl_target.setStyleSheet('color: %s;' % (GREEN if passed else RED))
        self.lbl_target.setText('Target: %d/42 corners, levels %s' %
                                (len(self._live_corners), levels))
        self._update_buttons()

    def _set_overlay(self, bgr: np.ndarray) -> None:
        image = np.ascontiguousarray(bgr)
        height, width = image.shape[:2]
        qimage = QImage(image.data, width, height, width * 3,
                        QImage.Format.Format_BGR888)
        self._last_overlay = QPixmap.fromImage(qimage.copy())
        self._rescale_overlay()

    def _rescale_overlay(self) -> None:
        if self._last_overlay is not None:
            self.overlay.setPixmap(self._last_overlay.scaled(
                self.overlay.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._rescale_overlay()

    def _load_home(self) -> List[float]:
        point = getattr(self._cfg, 'calibration_point', None)
        raw = getattr(point, 'motor_angles_deg', []) if point else []
        try:
            values = [float(value) for value in raw]
        except (TypeError, ValueError):
            return []
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            return []
        return values

    def _config_snapshot(self) -> Dict:
        try:
            payload = asdict(self._cfg)
        except Exception:
            return {'snapshot_error': 'Configuration is not a dataclass.'}
        ssh = payload.get('ssh')
        if isinstance(ssh, dict):
            ssh['password'] = ''
        return payload

    def _create_paper_archive(self):
        return paper_archive.PaperCalibrationArchive.create(
            camera_info=self._camera_info,
            home=self._home,
            grid=calibration.CALIBRATION_GRID,
            model_path=self._model.path,
            model_provenance=self._model.provenance(),
            config_snapshot=self._config_snapshot())

    def _read_motors(self) -> Optional[List[float]]:
        state = self._get_state()
        if state is None:
            return None
        stamp = float(getattr(state, 'timestamp', 0.0) or 0.0)
        age = time.time() - stamp
        if stamp <= 0.0 or age < 0.0 or age > _STATE_MAX_AGE_S:
            return None
        values = []
        for joint_id in (1, 2, 3, 4):
            reading = state.joint(joint_id)
            if reading is None or not reading.valid:
                return None
            values.append(float(reading.angle_deg))
        return values

    def _robot_state_snapshot(self) -> Dict:
        state = self._get_state()
        rows = []
        if state is not None:
            for joint_id in (1, 2, 3, 4):
                joint = state.joint(joint_id)
                rows.append({
                    'joint_id': joint_id,
                    'online': bool(joint.online) if joint else False,
                    'angle_deg': joint.angle_deg if joint else None,
                    'speed_rpm': joint.speed_rpm if joint else None,
                    'torque_nm': joint.torque_nm if joint else None,
                })
        return {
            'captured_at': datetime.now().astimezone().isoformat(
                timespec='milliseconds'),
            'host_monotonic': time.monotonic(),
            'state_timestamp_unix': getattr(state, 'timestamp', None),
            'source': getattr(state, 'source', None),
            'bus_voltage_v': getattr(state, 'bus_voltage', None),
            'bus_current_a': getattr(state, 'bus_current', None),
            'latency_ms': getattr(state, 'latency_ms', None),
            'joints': rows,
        }

    def _move_home(self) -> None:
        if not self._home_set:
            QMessageBox.warning(self, self.windowTitle(),
                                'No valid calibration point in config.json.')
            return
        backend = self._get_backend()
        current = self._read_motors()
        if (backend is None or not backend.is_connected()
                or not backend.is_motion_enabled() or current is None):
            QMessageBox.warning(
                self, self.windowTitle(),
                'Connect the arm, lock joints, and wait for fresh J1-J4 data.')
            return
        target = {joint_id: self._home[joint_id - 1]
                  for joint_id in (1, 2, 3, 4)}
        ok, _reason = backend.validate_targets(target, self._get_state())
        allow_large = False
        if not ok:
            ok_soft, reason_soft = backend.validate_targets(
                target, self._get_state(), check_step=False)
            if not ok_soft:
                QMessageBox.warning(self, self.windowTitle(), reason_soft)
                return
            delta = max(abs(a - b) for a, b in zip(current, self._home))
            if not confirm(
                    self, 'Large Calibration Move',
                    'The saved point is %.1f deg away. Confirm the swept path '
                    'is clear before moving.' % delta,
                    estop_cb=backend.estop):
                return
            allow_large = True
        backend.command_angles(target, _SPEED_RPM, _ACCEL,
                               allow_large_step=allow_large)
        self.lbl_detail.setText('Moving to the saved calibration point...')

    def _on_start(self) -> None:
        if self._running:
            return
        backend = self._get_backend()
        if backend is None or not backend.is_connected():
            QMessageBox.warning(self, self.windowTitle(),
                                'Not connected - connect the arm first.')
            return
        if not backend.is_motion_enabled():
            QMessageBox.warning(self, self.windowTitle(),
                                'Motion disabled - click Lock Joints first.')
            return
        if not self._camera_ready or self._latest_gray is None:
            QMessageBox.warning(
                self, self.windowTitle(),
                'RGB-D/IMU camera is not ready: %s' % self._camera_error)
            return
        if time.monotonic() - self._last_packet_t > 1.0:
            QMessageBox.warning(self, self.windowTitle(),
                                'Camera stream is stale; wait for a fresh frame.')
            return
        current = self._read_motors()
        if current is None:
            QMessageBox.warning(self, self.windowTitle(),
                                'J1-J4 must all have fresh state data.')
            return
        if not self._home_set:
            QMessageBox.warning(self, self.windowTitle(),
                                'No valid calibration point in config.json.')
            return
        max_home_error = max(abs(a - b) for a, b in zip(current, self._home))
        if max_home_error > _AT_HOME_TOL_DEG:
            QMessageBox.warning(
                self, self.windowTitle(),
                'Arm is not at the calibration point (max error %.2f deg).'
                % max_home_error)
            return
        try:
            preflight = calibration.probe_frame(
                self._latest_gray.copy(), self._latest_depth.copy(),
                self._K, self._dist, self._model)
        except Exception as exc:
            QMessageBox.warning(self, self.windowTitle(),
                                'Preflight failed: %s' % exc)
            return
        if not preflight['ok']:
            reg = preflight.get('registration') or {}
            QMessageBox.warning(
                self, self.windowTitle(),
                'Preflight rejected: %d corners, levels %s, depth RMS %s '
                'mm. Keep all three strips centered and unobstructed.' % (
                    preflight['n_corners'], preflight['ok_levels'],
                    ('%.3f' % reg['rms_mm']) if reg.get('ok') else '--'))
            return
        if not confirm(
                self, 'Start Calibration',
                'The arm will visit %d tested poses at 5 r/min '
                '(J1 +/-30, J2/J3 +/-6, J4 +/-18 deg). Keep the target fixed, '
                'clear the swept volume, and keep Esc/E-stop available.' %
                len(calibration.CALIBRATION_GRID),
                estop_cb=backend.estop):
            return

        targets = []
        for delta in calibration.CALIBRATION_GRID:
            target = {joint_id: self._home[joint_id - 1] + delta[joint_id - 1]
                      for joint_id in (1, 2, 3, 4)}
            ok, reason = backend.validate_targets(
                target, self._get_state(), check_step=False)
            if not ok:
                QMessageBox.warning(
                    self, self.windowTitle(),
                    'Calibration trajectory outside soft limits: %s' % reason)
                return
            targets.append(target)
        try:
            archive = self._create_paper_archive()
        except Exception as exc:
            QMessageBox.critical(
                self, self.windowTitle(),
                'Paper archive could not be created; no motion was sent:\n%s'
                % exc)
            return
        self._targets = targets
        self._samples = []
        self._quality_rows = []
        self._result = None
        self._last_saved_path = ''
        self._formal_promoted = False
        self._archive = archive
        self._index = 0
        self._running = True
        self._phase = 'settling'
        self.txt_result.clear()
        self._update_buttons()
        self._start_step()
        self._timer.start()

    def _start_step(self) -> None:
        backend = self._get_backend()
        if backend is None or not backend.is_motion_enabled():
            self._abort('Connection lost or motion disabled.', go_home=False)
            return
        self._target = self._targets[self._index]
        try:
            backend.command_angles(self._target, _SPEED_RPM, _ACCEL)
        except Exception as exc:
            self._abort('Motion command rejected: %s' % exc, go_home=False)
            return
        self._phase = 'settling'
        self._settle_started = time.monotonic()
        self._stable_ticks = 0
        self._update_progress('Moving / settling...')

    def _tick(self) -> None:
        if self._test_running:
            self._tick_test()
            return
        if not self._running:
            return
        backend = self._get_backend()
        if (backend is None or not backend.is_connected()
                or not backend.is_motion_enabled()):
            self._abort('Connection lost or motion disabled.', go_home=False)
            return
        if self._phase == 'settling':
            self._tick_settling()
        elif self._phase == 'collecting':
            self._tick_collecting()

    def _tick_settling(self) -> None:
        state = self._get_state()
        motors = self._read_motors()
        if motors is None:
            if time.monotonic() - self._settle_started > _SETTLE_TIMEOUT_S:
                self._abort('Joint state became stale while settling.', False)
            return
        errors = [abs(motors[jid - 1] - self._target[jid])
                  for jid in (1, 2, 3, 4)]
        speeds = []
        for jid in (1, 2, 3, 4):
            speed = float(state.joint(jid).speed_rpm)
            speeds.append(abs(speed) if math.isfinite(speed) else 0.0)
        if max(errors) <= _ANGLE_TOL_DEG and max(speeds) <= _SPEED_TOL_RPM:
            self._stable_ticks += 1
        else:
            self._stable_ticks = 0
        if self._stable_ticks >= _STABLE_TICKS:
            self._begin_capture()
            return
        if time.monotonic() - self._settle_started > _SETTLE_TIMEOUT_S:
            if max(errors) <= 2.0:
                self._begin_capture()
            else:
                self._abort('Settle timeout: max target error %.2f deg.' %
                            max(errors), go_home=False)

    def _begin_capture(self) -> None:
        self._phase = 'collecting'
        self._capture_started = time.monotonic()
        self._gray_buffer.clear()
        self._depth_buffer.clear()
        self._accel_buffer.clear()
        self._packet_buffer.clear()
        self._update_progress('Collecting RGB-D + IMU burst...')

    def _tick_collecting(self) -> None:
        elapsed = time.monotonic() - self._capture_started
        gray = [frame for stamp, frame in self._gray_buffer
                if stamp >= self._capture_started]
        depth = [frame for stamp, frame in self._depth_buffer
                 if stamp >= self._capture_started]
        accel = [row for stamp, row in self._accel_buffer
                 if stamp >= self._capture_started]
        packets = [packet for stamp, packet in self._packet_buffer
                   if stamp >= self._capture_started]
        ready = (elapsed >= _CAPTURE_SECONDS
                 and len(gray) >= _MIN_GRAY_FRAMES
                 and len(depth) >= _MIN_DEPTH_FRAMES
                 and len(packets) >= _MIN_DEPTH_FRAMES
                 and len(accel) >= _MIN_ACCEL_ROWS)
        if not ready and elapsed < 4.0:
            return
        motors = self._read_motors()
        if motors is None:
            self._abort('Joint state stale at capture.', go_home=False)
            return
        try:
            sample, quality = calibration.build_sample(
                gray[-_MIN_GRAY_FRAMES:], depth[-_MIN_DEPTH_FRAMES:], accel,
                motors, self._K, self._dist, self._R_color_imu, self._model)
        except Exception as exc:
            sample, quality = None, {
                'accepted': False, 'error': str(exc), 'motor': motors,
            }
        quality['pose_index'] = self._index
        quality['delta_motor'] = list(calibration.CALIBRATION_GRID[self._index])
        quality['target_motor'] = [self._target[jid]
                                   for jid in (1, 2, 3, 4)]
        self._quality_rows.append(quality)
        try:
            if self._archive is None:
                raise RuntimeError('Paper archive is unavailable.')
            self._archive.save_pose(
                index=self._index,
                packets=packets,
                quality=quality,
                robot_state=self._robot_state_snapshot(),
                target_motor=quality['target_motor'],
                delta_motor=quality['delta_motor'],
                depth_scale=float(self._camera_info.get(
                    'depth_scale_m_per_unit',
                    self._camera_info.get('depth_scale', 0.001))))
        except Exception as exc:
            self._abort(
                'Paper archive failed at pose %d: %s' %
                (self._index + 1, exc), go_home=True)
            return
        if sample is not None:
            self._samples.append(sample)
            reg = quality.get('registration') or {}
            self.lbl_detail.setText(
                'Accepted pose %d: %d corners, depth RMS %.3f mm' % (
                    self._index + 1, quality['n_corners'],
                    float(reg.get('rms_mm', float('nan')))))
        else:
            self.lbl_detail.setText(
                'Skipped pose %d: target/depth quality gate failed.' %
                (self._index + 1))
        self._index += 1
        self._update_count()
        if self._index >= len(self._targets):
            self._finish()
        else:
            self._start_step()

    def _finish(self) -> None:
        self._timer.stop()
        self._go_home_quietly()
        self._running = False
        self._phase = 'solving'
        self._update_buttons()
        self._update_progress('Solving anchors...')
        try:
            self._result = calibration.solve_samples(
                self._samples, self._K, self._dist, self._model)
        except Exception as exc:
            self._result = {
                'ok': False, 'method': 'ANCHOR_FUSION_3D',
                'n': len(self._samples), 'msg': str(exc),
                'promotion_allowed': False,
            }
        self._phase = 'idle'
        archive_error = ''
        try:
            self._save_artifacts()
        except Exception as exc:
            archive_error = str(exc)
            self._formal_promoted = False
            self._result['archive_error'] = archive_error
            self._result['promotion_allowed'] = False
            self._result['ok'] = False
            self._result['msg'] = (
                self._result.get('msg', '') +
                ' Paper archive/promotion failed: ' + archive_error).strip()
        self._show_result()
        self._update_buttons()
        self._update_progress('Done')
        if archive_error:
            QMessageBox.critical(self, self.windowTitle(),
                                 self._result.get('msg', archive_error))
        elif not self._result.get('ok'):
            QMessageBox.warning(self, self.windowTitle(),
                                self._result.get('msg', 'Calibration failed.'))

    def _show_result(self) -> None:
        result = self._result or {}
        lines = [
            'Status: %s' % ('PASS' if result.get('ok') else 'REJECTED'),
            'Method: %s' % result.get('method', '--'),
            'Accepted poses: %d/%d' % (len(self._samples), len(self._targets)),
        ]
        if result.get('reproj_rms_px') is not None:
            lines.extend([
                'Reprojection RMS: %.3f px' % result['reproj_rms_px'],
                'Depth RMS: %.3f mm' % result['depth_rms_mm'],
                'Gravity RMS: %.3f deg' % result['gravity_rms_deg'],
                'Camera translation norm: %.2f mm' %
                result['camera_translation_norm_mm'],
                'Jacobian condition: %.1f' % result['jacobian_condition'],
            ])
        if result.get('t') is not None:
            translation = np.asarray(result['t']) * 1000.0
            lines.append('T_J4_color translation [mm]: [%.3f, %.3f, %.3f]' %
                         tuple(translation))
        lines.extend(['', result.get('msg', '')])
        lines.append('Formal result: %s' % (
            'ACTIVE' if self._formal_promoted else 'NOT PROMOTED'))
        if self._archive is not None:
            lines.append('Paper archive: %s' % self._archive.root)
        self.txt_result.setPlainText('\n'.join(lines))
        self.lbl_detail.setText(result.get('msg', ''))

    def _artifact_header(self) -> Dict:
        return {
            'timestamp_iso': datetime.now().astimezone().isoformat(
                timespec='seconds'),
            'home_motors': self._home,
            'grid': calibration.CALIBRATION_GRID,
            'K_color': self._K,
            'dist_color': self._dist,
            'camera': self._camera_info,
            'model_path': self._model.path,
            'model_provenance': self._model.provenance(),
            'accepted_poses': len(self._samples),
            'paper_run_id': self._archive.run_id if self._archive else None,
            'paper_archive': self._archive.root if self._archive else None,
        }

    def _save_artifacts(self) -> None:
        if self._archive is None:
            raise RuntimeError('Paper archive is unavailable.')
        result = self._result or {}
        gates = result.get('numeric_gates') or {}
        numeric_pass = bool(
            result.get('ok') and result.get('promotion_allowed')
            and gates and all(bool(value) for value in gates.values()))
        status = 'PASS' if numeric_pass else 'REJECTED'
        header = self._artifact_header()
        run_path = self._archive.finalize(
            status=status,
            result=result,
            header=header,
            quality_rows=self._quality_rows,
            reason=result.get('msg', ''))
        self._last_saved_path = run_path
        self._formal_promoted = False
        if self._archive.should_promote(result):
            self._last_saved_path = self._archive.promote_formal(
                result=result, header=header)
            self._formal_promoted = True

    def _on_save_result(self) -> None:
        if not self._result:
            QMessageBox.information(self, self.windowTitle(),
                                    tr('No result to save'))
            return
        if not self._last_saved_path:
            self._save_artifacts()
        label = ('Formal result active' if self._formal_promoted
                 else 'Run archived; formal result unchanged')
        QMessageBox.information(
            self, self.windowTitle(),
            '%s:\n%s' % (label, self._last_saved_path))

    def _load_formal_result(self) -> Optional[Dict]:
        if (self._formal_promoted and self._result is not None
                and self._result.get('ok')
                and self._result.get('T_base_target') is not None):
            return self._result
        from ..core.apppaths import user_path
        import yaml

        path = user_path('calibration', 'handeye_result.yaml')
        try:
            with open(path, encoding='utf-8') as handle:
                payload = yaml.safe_load(handle) or {}
            result = payload.get('result') or {}
        except Exception:
            return None
        if not result.get('ok') or result.get('T_base_target') is None:
            return None
        return result

    def _solve_test_segment(
            self, start_motor: np.ndarray, start_xyz: np.ndarray,
            goal_xyz: np.ndarray, start_pitch: float, goal_pitch: float,
            inverse_board: np.ndarray, minimum_target_z_mm: float,
            stage: str) -> List[Dict[int, float]]:
        distance = float(np.linalg.norm(goal_xyz - start_xyz))
        pitch_change = abs(float(goal_pitch - start_pitch))
        initial = max(1, int(math.ceil(distance / _TEST_CARTESIAN_STEP_MM)),
                      int(math.ceil(pitch_change / 3.0)))
        for refinement in range(7):
            count = initial * (2 ** refinement)
            previous = start_motor.copy()
            rows: List[Dict[int, float]] = []
            failed = False
            for index in range(1, count + 1):
                fraction = index / count
                xyz = start_xyz + fraction * (goal_xyz - start_xyz)
                pitch = start_pitch + fraction * (goal_pitch - start_pitch)
                solution = kinematics.ik(
                    float(xyz[0]), float(xyz[1]), float(xyz[2]),
                    float(pitch), current=kinematics.motor_to_model(previous),
                    tol_mm=0.05)
                if solution is None:
                    failed = True
                    break
                motor = np.asarray(
                    kinematics.model_to_motor(solution), dtype=float)
                if (not np.all(np.isfinite(motor))
                        or any(not (self._cfg.angle_limits[jid][0]
                                    <= motor[jid - 1]
                                    <= self._cfg.angle_limits[jid][1])
                               for jid in (1, 2, 3, 4))
                        or float(np.max(np.abs(motor - previous)))
                        > _TEST_MAX_MOTOR_STEP_DEG):
                    failed = True
                    break
                # Audit the controller's joint-linear interpolation, not only
                # the Cartesian IK endpoints.
                for sample_index in range(1, 7):
                    sampled = previous + (sample_index / 6.0) * (motor - previous)
                    tcp_base = np.asarray(
                        kinematics.fk_from_motor(sampled)[:3], dtype=float)
                    tcp_target = (
                        inverse_board
                        @ np.r_[tcp_base / 1000.0, 1.0])[:3] * 1000.0
                    if float(tcp_target[2]) < minimum_target_z_mm:
                        failed = True
                        break
                if failed:
                    break
                rows.append({jid: float(motor[jid - 1])
                             for jid in (1, 2, 3, 4)})
                previous = motor
            if not failed and rows:
                return rows
        raise ValueError('%s path has no safe IK solution' % stage)

    def _build_test_leg(self, start_motor: np.ndarray, point_index: int,
                        board: np.ndarray) -> List[Dict[int, float]]:
        inverse_board = np.linalg.inv(board)
        normal = board[:3, 2] / np.linalg.norm(board[:3, 2])
        max_target_z = max(float(value[2])
                           for value in self._model.pts_mm.values())
        point_id = _TEST_POINT_IDS[point_index]
        point_target = np.asarray(self._model.pts_mm[point_id], dtype=float)
        point_base = (
            board @ np.r_[point_target / 1000.0, 1.0])[:3] * 1000.0
        high = point_base + _TEST_TRANSIT_MM * normal
        low = point_base + _TEST_CLEARANCE_MM * normal
        start_pose = kinematics.fk_from_motor(start_motor)
        start_xyz = np.asarray(start_pose[:3], dtype=float)
        start_pitch = float(start_pose[3])
        path: List[Dict[int, float]] = []

        if point_index == 0:
            first = self._solve_test_segment(
                start_motor, start_xyz, high, start_pitch, -90.0,
                inverse_board, max_target_z + 80.0, 'initial high transit')
            path.extend(first)
            current = np.asarray(list(first[-1].values()), dtype=float)
        else:
            previous_id = _TEST_POINT_IDS[point_index - 1]
            previous_target = np.asarray(
                self._model.pts_mm[previous_id], dtype=float)
            previous_base = (
                board @ np.r_[previous_target / 1000.0, 1.0])[:3] * 1000.0
            raised = previous_base + _TEST_TRANSIT_MM * normal
            raise_rows = self._solve_test_segment(
                start_motor, start_xyz, raised, start_pitch, -90.0,
                inverse_board,
                float(previous_target[2]) + _TEST_CLEARANCE_MM - 2.0,
                'raise from point %d' % previous_id)
            path.extend(raise_rows)
            current = np.asarray(list(raise_rows[-1].values()), dtype=float)
            traverse_rows = self._solve_test_segment(
                current, raised, high, -90.0, -90.0, inverse_board,
                max_target_z + 100.0,
                'high transit %d to %d' % (previous_id, point_id))
            path.extend(traverse_rows)
            current = np.asarray(
                list(traverse_rows[-1].values()), dtype=float)

        lower_rows = self._solve_test_segment(
            current, high, low, -90.0, -90.0, inverse_board,
            float(point_target[2]) + _TEST_CLEARANCE_MM - 2.0,
            'lower to point %d hover' % point_id)
        path.extend(lower_rows)
        return path

    def _on_test_start(self) -> None:
        if self._running or self._test_running:
            return
        backend = self._get_backend()
        current = self._read_motors()
        if (backend is None or not backend.is_connected()
                or not backend.is_motion_enabled() or current is None):
            QMessageBox.warning(
                self, self.windowTitle(),
                'Connect the arm, lock joints, and wait for fresh J1-J4 data.')
            return
        if not self._home_set:
            QMessageBox.warning(self, self.windowTitle(),
                                'No valid calibration point in config.json.')
            return
        home_error = max(abs(a - b) for a, b in zip(current, self._home))
        if home_error > _AT_HOME_TOL_DEG:
            QMessageBox.warning(
                self, self.windowTitle(),
                'Move to Calibration Point before testing (error %.2f deg).'
                % home_error)
            return
        result = self._load_formal_result()
        if result is None:
            QMessageBox.warning(
                self, self.windowTitle(),
                'No valid formal handeye_result.yaml. Run calibration first.')
            return
        board = np.asarray(result['T_base_target'], dtype=float)
        if board.shape != (4, 4) or not np.all(np.isfinite(board)):
            QMessageBox.warning(self, self.windowTitle(),
                                'Formal target pose is invalid.')
            return
        if not confirm(
                self, 'Test Calibration Points',
                'The arm will test points %s in order. Each point stops %.0f mm '
                'above its surface; no automatic contact descent is used. '
                'Click Continue for every next point. Keep the target fixed '
                'and the swept volume clear.'
                % (', '.join(str(value) for value in _TEST_POINT_IDS),
                   _TEST_CLEARANCE_MM),
                estop_cb=backend.estop):
            return

        try:
            plans: List[List[Dict[int, float]]] = []
            simulated = np.asarray(current, dtype=float)
            for point_index in range(len(_TEST_POINT_IDS)):
                leg = self._build_test_leg(simulated, point_index, board)
                for target in leg:
                    ok, reason = backend.validate_targets(
                        target, None, check_step=False)
                    if not ok:
                        raise ValueError(reason)
                plans.append(leg)
                simulated = np.asarray(list(leg[-1].values()), dtype=float)
        except Exception as exc:
            QMessageBox.warning(
                self, self.windowTitle(),
                'Point-test path audit failed before motion: %s' % exc)
            return

        self._test_board = board
        self._test_plans = plans
        self._test_running = True
        self._test_waiting_continue = False
        self._test_point_index = 0
        self._start_test_leg()
        self._timer.start()
        self._update_buttons()

    def _start_test_leg(self) -> None:
        self._test_queue = list(self._test_plans[self._test_point_index])
        self._command_next_test_waypoint()
        self._update_progress('Safe point-test motion...')

    def _command_next_test_waypoint(self) -> None:
        if not self._test_queue:
            point_id = _TEST_POINT_IDS[self._test_point_index]
            if self._test_point_index >= len(_TEST_POINT_IDS) - 1:
                self._test_running = False
                self._test_waiting_continue = False
                self._timer.stop()
                self.lbl_detail.setText(
                    'Point test completed at point 41. The arm remains at '
                    'the 30 mm safe hover position.')
                self.lbl_step.setText('Test point 12/12 (ID 41)')
                self.lbl_phase.setText('Point test complete')
                self.lbl_count.setText('12 points completed')
                self._update_buttons()
                return
            self._test_waiting_continue = True
            self._timer.stop()
            self.lbl_detail.setText(
                'Reached point %d at %.0f mm clearance. Verify alignment, '
                'then click Continue.' % (point_id, _TEST_CLEARANCE_MM))
            self._update_progress('Waiting for Continue')
            self._update_buttons()
            return
        backend = self._get_backend()
        if (backend is None or not backend.is_connected()
                or not backend.is_motion_enabled()):
            self._abort_test('Connection lost or motion disabled.', False)
            return
        self._test_target = self._test_queue.pop(0)
        backend.command_angles(
            self._test_target, _TEST_SPEED_RPM, _TEST_ACCEL,
            allow_large_step=False)
        self._test_settle_started = time.monotonic()
        self._test_stable_ticks = 0

    def _tick_test(self) -> None:
        if self._test_waiting_continue:
            return
        backend = self._get_backend()
        state = self._get_state()
        motors = self._read_motors()
        if (backend is None or not backend.is_connected()
                or not backend.is_motion_enabled()):
            self._abort_test('Connection lost or motion disabled.', False)
            return
        if motors is None or state is None:
            if time.monotonic() - self._test_settle_started > _SETTLE_TIMEOUT_S:
                self._abort_test('Joint state became stale during point test.',
                                 True)
            return
        errors = [abs(motors[jid - 1] - self._test_target[jid])
                  for jid in (1, 2, 3, 4)]
        speeds = []
        for jid in (1, 2, 3, 4):
            speed = float(state.joint(jid).speed_rpm)
            speeds.append(abs(speed) if math.isfinite(speed) else 0.0)
        if max(errors) <= _ANGLE_TOL_DEG and max(speeds) <= _SPEED_TOL_RPM:
            self._test_stable_ticks += 1
        else:
            self._test_stable_ticks = 0
        if self._test_stable_ticks >= _STABLE_TICKS:
            self._command_next_test_waypoint()
            return
        if time.monotonic() - self._test_settle_started > _SETTLE_TIMEOUT_S:
            self._abort_test(
                'Point-test settle timeout: max error %.2f deg.' % max(errors),
                True)

    def _on_test_continue(self) -> None:
        if not self._test_running or not self._test_waiting_continue:
            return
        self._test_point_index += 1
        self._test_waiting_continue = False
        self._start_test_leg()
        self._timer.start()
        self._update_buttons()

    def _abort_test(self, reason: str, estop: bool) -> None:
        self._timer.stop()
        backend = self._get_backend()
        if estop and backend is not None:
            try:
                backend.estop()
            except Exception:
                pass
        self._test_running = False
        self._test_waiting_continue = False
        self._test_queue = []
        self.lbl_detail.setText(reason)
        self.lbl_phase.setText('Point test stopped')
        self._update_buttons()

    def _on_stop(self) -> None:
        if self._running:
            self._abort('Calibration stopped by user.', go_home=True)
        elif self._test_running:
            self._abort_test(
                'Point test stopped by user.',
                estop=not self._test_waiting_continue)

    def _abort(self, reason: str, go_home: bool) -> None:
        self._timer.stop()
        self._running = False
        self._phase = 'idle'
        if go_home:
            self._go_home_quietly()
        if self._archive is not None and not self._archive.finalized:
            try:
                status = ('ERROR' if 'failed' in reason.lower()
                          else 'ABORTED')
                self._last_saved_path = self._archive.finalize(
                    status=status,
                    result=self._result,
                    header=self._artifact_header(),
                    quality_rows=self._quality_rows,
                    reason=reason)
            except Exception as exc:
                reason = '%s Archive finalization also failed: %s' % (
                    reason, exc)
        self._formal_promoted = False
        self.lbl_detail.setText(reason)
        self._update_buttons()
        self._update_progress('Stopped')

    def _go_home_quietly(self) -> None:
        backend = self._get_backend()
        if (backend is not None and backend.is_connected()
                and backend.is_motion_enabled() and self._home_set):
            try:
                backend.command_angles(
                    {jid: self._home[jid - 1] for jid in (1, 2, 3, 4)},
                    _SPEED_RPM, _ACCEL)
            except Exception:
                pass

    def _update_progress(self, phase: str = '') -> None:
        if self._test_running:
            current = self._test_point_index + 1
            self.lbl_step.setText(
                'Test point %d/%d (ID %d)' % (
                    current, len(_TEST_POINT_IDS),
                    _TEST_POINT_IDS[self._test_point_index]))
            self.lbl_phase.setText(phase or tr('Idle'))
            self.lbl_count.setText('Sequence: %s' % ', '.join(
                str(value) for value in _TEST_POINT_IDS))
            return
        total = len(calibration.CALIBRATION_GRID)
        current = min(self._index + 1, total) if self._running else self._index
        self.lbl_step.setText('Step %d/%d' % (current, total))
        self.lbl_phase.setText(phase or tr('Idle'))
        self._update_count()

    def _update_count(self) -> None:
        self.lbl_count.setText('Accepted: %d' % len(self._samples))

    def _update_buttons(self) -> None:
        # A transient corner count no longer greys out Start. Clicking it runs
        # preflight and reports the exact missing camera/target/arm condition.
        busy = self._running or self._test_running
        self.btn_start.setEnabled((not busy) and self._home_set)
        self.btn_start.setToolTip(
            'Run preflight; unmet conditions are reported explicitly.')
        self.btn_stop.setEnabled(busy)
        self.btn_home.setEnabled((not busy) and self._home_set)
        self.btn_save.setEnabled((not busy) and self._result is not None)
        self.btn_test.setEnabled((not busy) and self._home_set)
        self.btn_test.setToolTip(
            'Guided safe-hover test: 0, 6, 7, 13, 14, 20, 21, 27, 28, '
            '34, 35, 41.')
        self.btn_continue.setEnabled(
            self._test_running and self._test_waiting_continue)

    def keyPressEvent(self, event) -> None:  # noqa: N802
        if event.key() == Qt.Key.Key_Escape:
            backend = self._get_backend()
            if backend is not None:
                try:
                    backend.estop()
                except Exception:
                    pass
            if self._running:
                self._abort('E-stop pressed - calibration aborted.', False)
            if self._test_running:
                self._abort_test('E-stop pressed - point test aborted.', False)
            event.accept()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._running or self._test_running:
            backend = self._get_backend()
            if backend is not None:
                try:
                    backend.estop()
                except Exception:
                    pass
            if self._running:
                self._abort('Dialog closed - calibration aborted.', False)
            if self._test_running:
                self._abort_test('Dialog closed - point test aborted.', False)
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(1800)
            self._worker = None
        self._camera_ready = False
        super().closeEvent(event)
