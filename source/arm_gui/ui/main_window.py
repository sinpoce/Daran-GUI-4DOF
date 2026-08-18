# -*- coding: utf-8 -*-
"""主窗口：工具栏（使能/回零/急停）+ 功能菜单 + 连接 Dock + 关节面板 + 遥测 Dock。

后端生命周期完全由本类管理：
* 连接时惰性 import CanBackend，把 AppConfig.safety + joints 灌入
  backend.safety；
* 切换/断开前先 shutdown() 旧后端；
* 所有 backend 信号来自工作线程，Qt 自动按队列送达 GUI 线程，
  槽函数内不做任何阻塞调用。

安全原则：运动默认禁止；急停在已连接时永远可用（无需确认，Esc 快捷键）；
目标下发前先 validate_targets() 软限位/步长校验，超限整条拒绝。
"""
from __future__ import annotations

import os
import math
import time
from typing import Dict, Optional

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtGui import QGuiApplication, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QDockWidget, QLabel, QMainWindow, QPushButton, QSizePolicy, QToolBar,
    QWidget,
)

from ..core.backend_base import ArmBackend, SafetyLimits
from ..core.config import AppConfig
from ..core.state import ArmState
from ..i18n import register, tr
from ..version import APP_VERSION
from .camera_panel import CameraPanel
from .connection_panel import ConnectionPanel
from .joint_panel import JointPanel
from .style import FG_FAINT, GREEN, YELLOW
from .telemetry_panel import TelemetryPanel
from .widgets import StatusLed, confirm, repolish

# 警告文案常量存英文原文，使用处再包 tr()：模块导入发生在 main.py 设定语言之前，
# 且 register() 在文件底部，此处不能在导入期求值 tr()
_ZERO_WARNING = (
    'About to command all joints to move to the 0° position.\n\n'
    'The arm will execute a trapezoidal trajectory at the configured '
    'speed/acceleration.\n'
    'Please make sure the motion path is clear of people and obstacles.\n\n'
    'Proceed with homing?'
)

_SET_SELECTED_ZERO_WARNING = (
    'About to persistently set the current positions of these arm joints '
    'as 0°:\n%s\n\n'
    'Each selected joint may briefly lose holding torque while its drive '
    'configuration is written. Mechanically support the arm.\n\n'
    'This changes the joint coordinate reference. Existing teach recordings, '
    'calibration data, and saved points may no longer reproduce correctly.\n\n'
    'Proceed with setting the selected zero points?'
)

_RELEASE_WARNING = (
    'About to release all joints (switch to standby mode)!\n\n'
    'The joints will lose holding torque and the arm will sag under '
    'gravity —\n'
    'hold the arm steady first, or make sure it is in a stable/supported '
    'pose.\n'
    'Once released you can position it by hand; click "Lock Joints" to '
    'restore closed-loop before moving again.\n\n'
    'Proceed with release?'
)


class MainWindow(QMainWindow):
    """DrEmpower 机械臂控制台主窗口。"""

    # 示教回放排队等待：锁定/使能是异步的（后端要写 requested_state 再读回核验），
    # 补发请求的最小间隔与整体放弃时限。间隔必须远大于遥测周期（50ms），
    # 否则每帧补发一次会把工作线程的命令队列灌满、回放永远排不上。
    _TEACH_RETRY_PERIOD_S = 1.5
    _TEACH_START_TIMEOUT_S = 12.0
    # 参考 positions_done() 会无限阻塞；GUI 用遥测做可取消的到位判定。
    _TEACH_PREPOSITION_TOL_DEG = 1.0
    _TEACH_PREPOSITION_SPEED_TOL_RPM = 1.0
    _TEACH_PREPOSITION_STABLE_SAMPLES = 2
    _TEACH_MIN_FRAME_INTERVAL_S = 0.02
    _TEACH_TRAJECTORY_IDLE_POLL_MS = 20
    _TEACH_TRAJECTORY_DRAIN_TIMEOUT_S = 5.0
    _TEACH_TELEMETRY_TIMEOUT_S = 0.5
    _SET_ZERO_TELEMETRY_TIMEOUT_S = 1.0
    _SET_ZERO_STATIONARY_SPEED_RPM = 0.5

    def __init__(self, cfg: AppConfig, config_path: Optional[str] = None):
        super().__init__()
        self.cfg = cfg
        self._config_path = config_path
        self.backend: Optional[ArmBackend] = None
        self._connected = False
        self._last_state: Optional[ArmState] = None
        self._joints_mode = 2          # 1=待机(释放) 2=闭环(锁定)；上电默认闭环
        self._calib_dialog = None      # 手眼标定对话框（非模态）引用，避免被 GC
        self._point_recorder_dialog = None
        self._teach_dialog = None
        self._set_zero_dialog = None
        self._last_state_received_monotonic = 0.0
        self._teach_playback_timer = QTimer(self)
        self._teach_playback_timer.setSingleShot(True)
        self._teach_playback_timer.timeout.connect(self._on_teach_playback_tick)
        self._teach_playback_frames: list[dict] = []
        self._teach_playback_index = 0
        self._teach_playback_speed = 8.0
        self._teach_playback_accel = 10.0
        self._teach_playback_loops = 1
        self._teach_playback_loop_index = 0
        # ``draining`` means the final sample has been submitted and the UI is
        # waiting for the backend worker to finish sending it.  It is still an
        # active playback phase: no manual command may interleave with it.
        self._teach_playback_phase = 'idle'  # idle | preposition | tracking | draining
        self._teach_trajectory_token: Optional[int] = None
        self._teach_trajectory_drain_deadline = 0.0
        self._teach_requires_gripper = False
        self._teach_last_valid_telemetry_ts = 0.0
        self._teach_preposition_target: Dict[int, float] = {}
        self._teach_preposition_gripper: Optional[float] = None
        self._teach_preposition_deadline = 0.0
        self._teach_preposition_stable = 0
        self._teach_playback_request: Optional[tuple[list[dict], int, float, float]] = None
        # 排队等待期间的节流/超时（monotonic 秒）：锁定/使能请求每次只补发一条，
        # 绝不随遥测帧（20Hz）重复入队，否则工作线程会被 mode 命令淹没
        self._teach_lock_req_ts = 0.0
        self._teach_enable_req_ts = 0.0
        self._teach_request_deadline = 0.0

        self.setWindowTitle('%s v%s' % (
            tr('DrEmpower Arm Console'), APP_VERSION))
        self.resize(1400, 980)      # 中央区含关节卡片 + 下方相机画面

        self._build_toolbar()
        self._build_central()
        self._build_docks()
        self._build_menus()
        self._build_statusbar()

        # Esc = 急停（应用级快捷键，任何焦点下均生效）
        self._estop_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self._estop_shortcut.setContext(Qt.ShortcutContext.ApplicationShortcut)
        self._estop_shortcut.activated.connect(self._on_estop)

        self._restore_geometry()
        self._apply_mode_style()
        self._update_ui_state()
        self.telemetry.append_log('info', tr(
            'Console started (connect to take control; Esc E-stop is always '
            'available; after E-stop/release, click "Lock Joints" to resume '
            'control)'))

    # ================= 界面构建 =================
    def _build_toolbar(self) -> None:
        tb = QToolBar(tr('Main Toolbar'), self)
        tb.setMovable(False)
        tb.setFloatable(False)
        self.addToolBar(tb)

        # 连接状态 LED + 文字
        self.led = StatusLed(FG_FAINT)
        self.conn_lbl = QLabel(tr('Not Connected'))
        self.conn_lbl.setObjectName('dimLabel')
        tb.addWidget(self._wrap_margin(self.led, left=8))
        tb.addWidget(self._wrap_margin(self.conn_lbl, right=8))
        tb.addSeparator()

        # 回到零位
        self.btn_home = QPushButton(tr('Go to Zero'))
        self.btn_home.setToolTip(tr('Move all joints to 0° (motion enable required)'))
        self.btn_home.clicked.connect(self._on_go_home)
        tb.addWidget(self._wrap_margin(self.btn_home, left=6))

        tb.addSeparator()

        # 锁定/释放关节（闭环/待机，广播；不经运动使能门，各自把关）
        self.btn_lock = QPushButton(tr('Lock Joints'))
        self.btn_lock.setToolTip(tr('All joints enter closed-loop mode: hold current position (joints must be locked before motion)'))
        self.btn_lock.clicked.connect(self._on_lock_joints)
        tb.addWidget(self._wrap_margin(self.btn_lock, left=6))

        self.btn_release = QPushButton(tr('Release Joints'))
        self.btn_release.setToolTip(tr('All joints enter standby mode: unloaded for manual positioning (the arm will sag; confirmation required)'))
        self.btn_release.clicked.connect(self._on_release_joints)
        tb.addWidget(self._wrap_margin(self.btn_release, left=6))

        # 弹性空隙
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding,
                             QSizePolicy.Policy.Preferred)
        tb.addWidget(spacer)

        # 急停（大红，无需确认，Esc）
        self.btn_estop = QPushButton(tr('E-STOP (Esc)'))
        self.btn_estop.setObjectName('estopButton')
        self.btn_estop.setMinimumHeight(40)
        self.btn_estop.setToolTip(tr('Broadcast E-stop immediately (no confirmation; unavailable when not connected)'))
        self.btn_estop.clicked.connect(self._on_estop)
        tb.addWidget(self._wrap_margin(self.btn_estop, right=8))

    @staticmethod
    def _wrap_margin(w: QWidget, left: int = 0, right: int = 0) -> QWidget:
        """工具栏小控件加边距的包装。"""
        from PySide6.QtWidgets import QHBoxLayout
        box = QWidget()
        lay = QHBoxLayout(box)
        lay.setContentsMargins(left, 0, right, 0)
        lay.addWidget(w)
        return box

    def _build_central(self) -> None:
        """中央区：上=关节卡片区，下=相机画面（固定布局，不可增删）。"""
        self.joint_panel = JointPanel(self.cfg.joints, self.cfg.safety, self)
        self.joint_panel.move_requested.connect(self._on_move_requested)
        self.joint_panel.jog_requested.connect(self._on_jog_requested)
        self.joint_panel.cartesian_requested.connect(self._on_cartesian_requested)
        self.joint_panel.gripper_move_requested.connect(self._on_gripper_move)
        self.joint_panel.gripper_zero_requested.connect(self._on_gripper_zero)
        self.camera_view = CameraPanel(self.cfg, self)
        self.camera_view.setMinimumHeight(260)

        from PySide6.QtWidgets import QVBoxLayout
        central = QWidget()
        lay = QVBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(self.joint_panel, 3)
        lay.addWidget(self.camera_view, 2)
        self.setCentralWidget(central)

    def _build_docks(self) -> None:
        no_features = QDockWidget.DockWidgetFeature.NoDockWidgetFeatures

        # 左：连接（固定，不可关闭/移动/浮动；限宽给中央区和日志让空间）
        self.conn_panel = ConnectionPanel(self.cfg, self)
        self.conn_panel.connect_requested.connect(self._on_connect_requested)
        self.conn_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.conn_panel.setMaximumWidth(300)
        self.dock_conn = QDockWidget(tr('Connection'), self)
        self.dock_conn.setObjectName('dockConn')
        self.dock_conn.setWidget(self.conn_panel)
        self.dock_conn.setFeatures(no_features)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.dock_conn)

        # 右：遥测 + 日志（固定；加宽保证日志可读）
        self.telemetry = TelemetryPanel(self)
        self.telemetry.setMinimumWidth(380)
        self.dock_tele = QDockWidget(tr('Telemetry'), self)
        self.dock_tele.setObjectName('dockTele')
        self.dock_tele.setWidget(self.telemetry)
        self.dock_tele.setFeatures(no_features)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_tele)
        # 初始宽度分配：连接 260 / 遥测 420（中央区优先保证控件不被挤压）
        self.resizeDocks([self.dock_conn, self.dock_tele], [260, 420],
                         Qt.Orientation.Horizontal)

    def _build_menus(self) -> None:
        """菜单栏：功能菜单 + 语言选择（布局固定，无视图开关）。"""
        from PySide6.QtGui import QAction, QActionGroup
        from ..i18n import LANGUAGES, get_language

        # 功能菜单：把标定、点位记录和示教入口集中到一个可扩展的下拉菜单。
        # 顶层使用通用的“菜单”名称，避免把点位记录/示教误归类为标定。
        m_calib = self.menuBar().addMenu(tr('Menu'))
        set_zero_tip = tr('Open the joint zero-setting tool')
        self._act_set_zero = QAction(tr('Set Zero…'), self)
        self._act_set_zero.setToolTip(set_zero_tip)
        self._act_set_zero.setStatusTip(set_zero_tip)
        self._act_set_zero.triggered.connect(self._on_open_set_zero)

        act_handeye = QAction(tr('Hand-Eye Calibration…'), self)
        act_handeye.triggered.connect(self._on_hand_eye_calibration)
        m_calib.addAction(act_handeye)
        self._act_handeye = act_handeye
        act_point_recorder = QAction(tr('Manual Point Recorder…'), self)
        act_point_recorder.triggered.connect(self._on_point_recorder)
        m_calib.addAction(act_point_recorder)
        self._act_point_recorder = act_point_recorder
        # v1.2 默认启用示教功能；配置为 false 时不挂菜单入口，且不
        # import teach_playback_dialog，从而可临时隔离整套示教流程。
        self._act_teach = None
        if self.cfg.ui.enable_teach_playback:
            act_teach = QAction(tr('Teach Playback…'), self)
            act_teach.triggered.connect(self._on_teach_playback)
            m_calib.addAction(act_teach)
            self._act_teach = act_teach
        # 设零会持久化且短暂卸载关节，放在菜单底部并与普通入口隔开，
        # 降低误触风险。
        m_calib.addSeparator()
        m_calib.addAction(self._act_set_zero)

        m_lang = self.menuBar().addMenu(tr('Language'))
        group = QActionGroup(self)
        group.setExclusive(True)
        self._lang_actions = {}
        for code, label in LANGUAGES.items():
            act = QAction(label, self)
            act.setCheckable(True)
            act.setChecked(get_language() == code)
            act.triggered.connect(lambda _checked=False, c=code:
                                  self._on_language_selected(c))
            group.addAction(act)
            m_lang.addAction(act)
            self._lang_actions[code] = act

    def _on_language_selected(self, code: str) -> None:
        """切换语言：写入配置，确认后重启应用生效。"""
        from ..i18n import get_language
        if code == get_language():
            return
        self.cfg.ui.language = code
        self._save_config()
        if confirm(self, tr('Switch Language'),
                   tr('The new language takes effect after restart. Restart now?')):
            self._restart_app()

    def _restart_app(self) -> None:
        import subprocess
        import sys as _sys
        if getattr(_sys, 'frozen', False):
            # In a PyInstaller build sys.executable is already ArmGUI.exe.
            # Passing argv[0] again would make the EXE receive its own path as
            # an unexpected positional argument and the restart would fail in
            # argparse before the new window can be shown.
            command = [_sys.executable] + _sys.argv[1:]
        else:
            script = os.path.abspath(_sys.argv[0])
            command = [_sys.executable, script] + _sys.argv[1:]
        subprocess.Popen(command)
        self.close()

    def _on_hand_eye_calibration(self) -> None:
        """打开手眼标定对话框（非模态）。

        相机独占：标定要开自己的 RealSense 会话，进入前先停预览释放相机。
        对话框惰性 import（构造不触碰 cv2/pyrealsense）；非模态 show() 保证
        Esc 全局急停仍可用。保留 self._calib_dialog 引用避免被 GC，关闭时清引用；
        未连接/未使能也允许打开（对话框内部自校验并提示）。
        """
        # 已打开则前置激活，避免重复实例化
        dlg = self._calib_dialog
        if dlg is not None:
            dlg.raise_()
            dlg.activateWindow()
            return
        from .calibration_dialog import CalibrationDialog
        self.camera_view.stop()        # 先停预览，再让标定接管相机
        dlg = CalibrationDialog(
            self.cfg,
            get_backend=lambda: self.backend,
            get_state=lambda: self._last_state,
            camera_panel=self.camera_view,
            parent=self,
        )
        dlg.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        dlg.finished.connect(self._on_calib_dialog_finished)
        self._calib_dialog = dlg
        dlg.show()

    def _on_calib_dialog_finished(self, _result: int = 0) -> None:
        """标定对话框关闭：清引用（对话框自带 WA_DeleteOnClose 会自毁）。"""
        self._calib_dialog = None

    def _on_point_recorder(self) -> None:
        """Open the read-only, Enter-to-record manual point collector."""
        dlg = self._point_recorder_dialog
        if dlg is not None:
            dlg.raise_()
            dlg.activateWindow()
            return
        from .point_recorder_dialog import PointRecorderDialog
        dlg = PointRecorderDialog(
            get_state=lambda: self._last_state,
            get_joints_mode=lambda: self._joints_mode,
            get_connected=lambda: self._connected,
            lock_joints=self._on_lock_joints,
            release_joints=self._on_release_joints,
            parent=self,
        )
        dlg.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        dlg.finished.connect(self._on_point_recorder_finished)
        self._point_recorder_dialog = dlg
        if self._last_state is not None:
            dlg.observe_state(self._last_state)
        dlg.show()

    def _on_point_recorder_finished(self, _result: int = 0) -> None:
        self._point_recorder_dialog = None

    def _on_open_set_zero(self) -> None:
        """Open the joint-selective persistent-zero tool.

        The menu item is navigation only: opening the dialog never sends a
        drive command.  The dialog deliberately receives only ``cfg.joints``
        (J1-J4), not the separately configured gripper motor.
        """
        dlg = self._set_zero_dialog
        if dlg is not None:
            dlg.raise_()
            dlg.activateWindow()
            return
        from .set_zero_dialog import SetZeroDialog
        dlg = SetZeroDialog(
            joints=[(joint.joint_id, joint.name) for joint in self.cfg.joints],
            get_connected=lambda: self._connected,
            get_motion_enabled=lambda: (
                self.backend is not None
                and self.backend.is_motion_enabled()),
            get_teach_busy=self._is_teach_busy,
            get_zero_busy=lambda: (
                self.backend is not None
                and self.backend.is_setting_zero()),
            parent=self,
        )
        dlg.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        dlg.set_zero_requested.connect(self._on_set_selected_zero)
        dlg.finished.connect(self._on_set_zero_dialog_finished)
        self._set_zero_dialog = dlg
        if self._last_state is not None:
            dlg.observe_state(self._last_state)
        dlg.show()

    def _on_set_zero_dialog_finished(self, _result: int = 0) -> None:
        self._set_zero_dialog = None

    def _on_teach_playback(self) -> None:
        """Open teach dialog for manual drag recording and trajectory playback."""
        if not self.cfg.ui.enable_teach_playback:
            self.telemetry.append_log('warn', tr(
                'Teach playback is disabled (set ui.enable_teach_playback=true '
                'in config.json to re-enable)'))
            return
        dlg = self._teach_dialog
        if dlg is not None:
            dlg.raise_()
            dlg.activateWindow()
            return
        from .teach_playback_dialog import TeachPlaybackDialog
        dlg = TeachPlaybackDialog(
            get_connected=lambda: self._connected,
            get_joints_mode=lambda: self._joints_mode,
            lock_joints=self._set_joints_locked_direct,
            release_joints=self._set_joints_released_direct,
            joint_ids=self.cfg.joint_ids,
            default_speed=self.joint_panel.speed_rpm(),
            default_accel=self.joint_panel.accel(),
            parent=self,
        )
        dlg.playback_requested.connect(self._start_teach_playback)
        dlg.playback_stop_requested.connect(self._on_teach_playback_stop_clicked)
        dlg.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)
        dlg.finished.connect(self._on_teach_playback_finished)
        self._teach_dialog = dlg
        if self._last_state is not None:
            dlg.observe_state(self._last_state)
        dlg.show()

    def _on_teach_playback_finished(self, _result: int = 0) -> None:
        self._teach_dialog = None

    def _on_teach_playback_stop_clicked(self) -> None:
        self._stop_teach_playback(tr('Teach playback stopped by user'))

    def _teach_feedback(self, level: str, text: str) -> None:
        """Log to telemetry AND echo into the teach dialog.

        Playback feedback used to be log-only: when a click could not start
        playback the dialog itself stayed silent, which reads as "the button
        does nothing".
        """
        self.telemetry.append_log(level, text)
        dlg = self._teach_dialog
        if dlg is not None:
            dlg.set_playback_status(text, level)

    def _teach_set_active(self, active: bool) -> None:
        dlg = self._teach_dialog
        if dlg is not None:
            dlg.set_playback_active(active)
        # The main toolbar/joint panel live outside the teach dialog.  Refresh
        # them at every phase boundary so a queued playback is just as exclusive
        # as one which is already streaming.
        self._update_ui_state()

    def _is_teach_busy(self) -> bool:
        """Return whether teach owns the arm or is changing its control mode."""
        playback_busy = (
            self._teach_playback_request is not None
            or self._teach_playback_phase != 'idle'
            or self._teach_trajectory_token is not None
        )
        dlg = self._teach_dialog
        return playback_busy or bool(dlg is not None and dlg.is_recording)

    def _validate_teach_frames(self, frames: list[dict]) -> tuple[bool, str]:
        """Pre-check the recording against soft limits / configured joint IDs.

        Without this, an out-of-range recording is rejected frame by frame
        inside the backend — 20 refusals per second and an arm that never
        moves.  Fail once, up front, with the offending joint named.
        """
        known = set(self.cfg.joint_ids)
        limits = self.cfg.angle_limits
        previous_t = -1.0
        for idx, frame in enumerate(frames):
            joints = frame.get('joints') or {}
            if not joints:
                return False, tr('Teach playback rejected: sample %d has no joint data') % idx
            sample_ids = {int(jid) for jid in joints}
            if sample_ids != known:
                return False, (tr('Teach playback rejected: sample %d must contain joints %s')
                               % (idx, sorted(known)))
            timestamp = float(frame.get('t', 0.0))
            if (not math.isfinite(timestamp) or timestamp < 0.0
                    or (idx > 0 and timestamp <= previous_t)):
                return False, (tr('Teach playback rejected: sample %d has an invalid timestamp')
                               % idx)
            previous_t = timestamp
            for jid, angle in joints.items():
                if not math.isfinite(float(angle)):
                    return False, (tr('Teach playback rejected: sample %d has a non-finite angle')
                                   % idx)
                if int(jid) not in known:
                    return False, (tr('Teach playback rejected: unknown joint ID %s')
                                   % jid)
                lim = limits.get(int(jid))
                if lim is not None and not (lim[0] <= float(angle) <= lim[1]):
                    return False, (tr('Teach playback rejected: sample %d J%d %.2f° '
                                      'is outside the soft limit [%.1f, %.1f]°')
                                   % (idx, int(jid), float(angle), lim[0], lim[1]))
            gripper = frame.get('gripper')
            if gripper is not None and not math.isfinite(float(gripper)):
                return False, (tr('Teach playback rejected: sample %d has a non-finite gripper angle')
                               % idx)
        return True, ''

    def _start_teach_playback(self, frames: list[dict], loops: int,
                              speed: float, accel: float) -> None:
        """Queue the reference-library playback flow after lock/enable."""
        if self.backend is None or not self._connected:
            self._teach_feedback('warn', tr('Teach playback rejected: not connected'))
            return
        if len(frames) < 2:
            self._teach_feedback('warn', tr('Teach playback rejected: not enough samples'))
            return
        ok, reason = self._validate_teach_frames(frames)
        if not ok:
            self._teach_feedback('error', reason)
            return
        self._stop_teach_playback(clear_request=False)
        self._teach_playback_request = (
            list(frames), max(1, int(loops)), float(speed), float(accel)
        )
        # 补发节流与放弃时限一并复位（monotonic：不受系统时钟调整影响）
        self._teach_lock_req_ts = 0.0
        self._teach_enable_req_ts = 0.0
        self._teach_request_deadline = time.monotonic() + self._TEACH_START_TIMEOUT_S
        # Pending lock/enable is part of playback.  In particular, leave Stop
        # enabled and prevent a manual move from racing the eventual start.
        self._teach_set_active(True)
        self._teach_feedback(
            'info', tr('Teach playback queued: waiting for lock and motion enable'))
        self._try_start_teach_playback()

    def _try_start_teach_playback(self) -> None:
        """Attempt to start queued playback after async mode/enable transitions.

        Called from every telemetry frame (20Hz) while a request is pending, so
        the lock/enable requests themselves are throttled: re-queueing a joint
        mode switch per frame floods the backend worker (each one does a
        broadcast write, a hold command and a verify read-back) and playback
        would never get its turn — the "clicking playback does nothing" bug.
        """
        req = self._teach_playback_request
        if req is None:
            return
        if self.backend is None or not self._connected:
            self._abort_teach_request(tr('Teach playback stopped: disconnected'))
            return

        frames, loops, speed, accel = req
        now = time.monotonic()

        if self._joints_mode != 2:
            if now >= self._teach_request_deadline:
                self._abort_teach_request(
                    tr('Teach playback cancelled: joints did not reach closed loop '
                       'within %.0f s (check the drive error log, then lock manually)')
                    % self._TEACH_START_TIMEOUT_S)
                return
            if now - self._teach_lock_req_ts >= self._TEACH_RETRY_PERIOD_S:
                self._teach_lock_req_ts = now
                self._set_joints_locked_direct()
                self._teach_feedback(
                    'info', tr('Teach playback waiting: requested joint lock'))
            return
        if not self.backend.is_motion_enabled():
            if now >= self._teach_request_deadline:
                self._abort_teach_request(
                    tr('Teach playback cancelled: motion enable was not granted '
                       'within %.0f s (check the drive error log)')
                    % self._TEACH_START_TIMEOUT_S)
                return
            if now - self._teach_enable_req_ts >= self._TEACH_RETRY_PERIOD_S:
                self._teach_enable_req_ts = now
                self.backend.set_motion_enabled(True)
                self._teach_feedback(
                    'info', tr('Teach playback waiting: requested motion enable'))
            return

        self._teach_playback_request = None
        self._teach_playback_frames = list(frames)
        self._teach_playback_index = 0
        self._teach_playback_speed = float(speed)
        self._teach_playback_accel = float(accel)
        self._teach_playback_loops = max(1, int(loops))
        self._teach_playback_loop_index = 0
        self._teach_requires_gripper = any(
            frame.get('gripper') is not None for frame in frames)
        self._teach_last_valid_telemetry_ts = 0.0
        self._teach_trajectory_token = self.backend.begin_trajectory()
        if not self._teach_trajectory_token:
            self._abort_teach_request(
                tr('Teach playback rejected: backend has no streamed trajectory support'))
            return
        self._teach_set_active(True)
        self._begin_teach_loop()

    def _begin_teach_loop(self) -> None:
        """Reference step 1: trapezoidal move to sample zero, then wait."""
        if (self.backend is None or not self._connected
                or self._teach_trajectory_token is None
                or not self._teach_playback_frames):
            self._stop_teach_playback(
                tr('Teach playback stopped: disconnected'), hold_position=False)
            return
        first = self._teach_playback_frames[0]
        joints = {int(jid): float(angle)
                  for jid, angle in (first.get('joints') or {}).items()}
        gripper = first.get('gripper')
        gripper = None if gripper is None else float(gripper)
        self._teach_playback_phase = 'preposition'
        self._teach_playback_index = 0
        self._teach_preposition_target = joints
        self._teach_preposition_gripper = gripper
        self._teach_preposition_stable = 0

        # Give long moves enough time while retaining a finite/cancellable wait.
        max_delta = 0.0
        if self._last_state is not None:
            for jid, target in joints.items():
                reading = self._last_state.joint(jid)
                if reading is not None and reading.valid:
                    max_delta = max(max_delta, abs(target - reading.angle_deg))
        speed_deg_s = max(3.0, abs(self._teach_playback_speed) * 6.0)
        timeout_s = max(
            self._TEACH_START_TIMEOUT_S, max_delta / speed_deg_s * 2.0 + 3.0)
        self._teach_preposition_deadline = time.monotonic() + timeout_s
        self.backend.command_trajectory_start(
            self._teach_trajectory_token, joints,
            self._teach_playback_speed, self._teach_playback_accel,
            gripper_deg=gripper)
        self._teach_feedback(
            'info', tr('Teach loop %d/%d: moving to the first sample')
            % (self._teach_playback_loop_index + 1, self._teach_playback_loops))

    def _observe_teach_preposition(self, state: ArmState) -> None:
        """Non-blocking replacement for the vendor positions_done() busy loop."""
        if self._teach_playback_phase != 'preposition':
            return
        if (self.backend is None or not self._connected
                or not self.backend.is_motion_enabled()):
            self._stop_teach_playback(
                tr('Teach playback stopped: motion not enabled'),
                hold_position=False)
            return
        if time.monotonic() >= self._teach_preposition_deadline:
            self._stop_teach_playback(
                tr('Teach playback cancelled: first sample was not reached in time'))
            return

        arrived = True
        for jid, target in self._teach_preposition_target.items():
            reading = state.joint(jid)
            if (reading is None or not reading.valid
                    or abs(reading.angle_deg - target)
                    > self._TEACH_PREPOSITION_TOL_DEG
                    or abs(reading.speed_rpm)
                    > self._TEACH_PREPOSITION_SPEED_TOL_RPM):
                arrived = False
                break
        if arrived and self._teach_preposition_gripper is not None:
            reading = state.gripper
            if (reading is None or not reading.valid
                    or abs(reading.angle_deg - self._teach_preposition_gripper)
                    > self._TEACH_PREPOSITION_TOL_DEG
                    or abs(reading.speed_rpm)
                    > self._TEACH_PREPOSITION_SPEED_TOL_RPM):
                arrived = False

        self._teach_preposition_stable = (
            self._teach_preposition_stable + 1 if arrived else 0)
        if self._teach_preposition_stable < self._TEACH_PREPOSITION_STABLE_SAMPLES:
            return
        self._teach_playback_phase = 'tracking'
        self._teach_playback_index = 0
        self._teach_feedback(
            'info', tr('Teach loop %d/%d: first sample reached; tracking started')
            % (self._teach_playback_loop_index + 1, self._teach_playback_loops))
        # The same snapshot which proved arrival is also the first tracking
        # health sample.  Validate it before submitting the first dense point.
        self._observe_teach_tracking_health(state)
        if self._teach_playback_phase == 'tracking':
            self._on_teach_playback_tick()

    def _on_teach_playback_tick(self) -> None:
        if self._teach_playback_phase == 'draining':
            self._wait_for_teach_trajectory_idle()
            return
        if self._teach_playback_phase != 'tracking':
            return
        if self.backend is None or not self._connected:
            self._stop_teach_playback(
                tr('Teach playback stopped: disconnected'), hold_position=False)
            return
        if self._teach_playback_index >= len(self._teach_playback_frames):
            # The GUI timer only says the recorded dwell elapsed.  The backend
            # may still have the coalesced final point pending or in-flight.
            self._teach_playback_phase = 'draining'
            self._teach_trajectory_drain_deadline = (
                time.monotonic() + self._TEACH_TRAJECTORY_DRAIN_TIMEOUT_S)
            self._wait_for_teach_trajectory_idle()
            return
        if not self.backend.is_motion_enabled():
            self._stop_teach_playback(
                tr('Teach playback stopped: motion not enabled'),
                hold_position=False)
            return
        if (self._teach_last_valid_telemetry_ts <= 0.0
                or time.monotonic() - self._teach_last_valid_telemetry_ts
                > self._TEACH_TELEMETRY_TIMEOUT_S):
            self._stop_teach_playback(
                tr('Teach playback stopped: required telemetry timed out'))
            return

        frame = self._teach_playback_frames[self._teach_playback_index]
        joints = {int(jid): float(a) for jid, a in (frame.get('joints') or {}).items()}
        gripper = frame.get('gripper')
        gripper = None if gripper is None else float(gripper)

        cur_t = float(frame.get('t', 0.0))
        if self._teach_playback_index + 1 < len(self._teach_playback_frames):
            nxt_t = float(self._teach_playback_frames[
                self._teach_playback_index + 1].get('t', cur_t + 0.05))
            dt = max(self._TEACH_MIN_FRAME_INTERVAL_S, nxt_t - cur_t)
        elif self._teach_playback_index > 0:
            prev_t = float(self._teach_playback_frames[
                self._teach_playback_index - 1].get('t', cur_t - 0.05))
            dt = max(self._TEACH_MIN_FRAME_INTERVAL_S, cur_t - prev_t)
        else:
            dt = 0.05
        bandwidth = min(299.0, 1.0 / (2.0 * dt))
        if self._teach_trajectory_token is None:
            self._stop_teach_playback(
                tr('Teach playback stopped: trajectory session lost'))
            return
        self.backend.command_trajectory_point(
            self._teach_trajectory_token, joints, bandwidth,
            gripper_deg=gripper)

        self._teach_playback_index += 1
        # Keep the final sample active for one recorded interval before
        # completing/repositioning the loop.
        interval_ms = max(20, int(round(dt * 1000.0)))
        self._teach_playback_timer.start(interval_ms)

    def _wait_for_teach_trajectory_idle(self) -> None:
        """Finish/loop only after the final point left the backend worker."""
        if self._teach_playback_phase != 'draining':
            return
        backend = self.backend
        token = self._teach_trajectory_token
        if backend is None or not self._connected:
            self._stop_teach_playback(
                tr('Teach playback stopped: disconnected'), hold_position=False)
            return
        if token is None:
            self._stop_teach_playback(
                tr('Teach playback stopped: trajectory session lost'))
            return
        if not backend.is_motion_enabled():
            self._stop_teach_playback(
                tr('Teach playback stopped: motion not enabled'),
                hold_position=False)
            return
        if (self._teach_last_valid_telemetry_ts <= 0.0
                or time.monotonic() - self._teach_last_valid_telemetry_ts
                > self._TEACH_TELEMETRY_TIMEOUT_S):
            self._stop_teach_playback(
                tr('Teach playback stopped: required telemetry timed out'))
            return
        if backend.is_trajectory_idle(token):
            self._finish_teach_loop()
            return
        if time.monotonic() >= self._teach_trajectory_drain_deadline:
            self._stop_teach_playback(
                tr('Teach playback stopped: final trajectory point did not drain'))
            return
        self._teach_playback_timer.start(self._TEACH_TRAJECTORY_IDLE_POLL_MS)

    def _observe_teach_tracking_health(self, state: ArmState) -> None:
        """Stop-and-hold on the first invalid required tracking reading."""
        if self._teach_playback_phase not in ('tracking', 'draining'):
            return
        for jid in self.cfg.joint_ids:
            reading = state.joint(jid)
            if (reading is None or not reading.valid
                    or not math.isfinite(float(reading.angle_deg))):
                self._stop_teach_playback(
                    tr('Teach playback stopped: required joint J%d telemetry '
                       'lost or invalid') % jid)
                return
        if self._teach_requires_gripper:
            reading = state.gripper
            if (reading is None or not reading.valid
                    or not math.isfinite(float(reading.angle_deg))):
                self._stop_teach_playback(
                    tr('Teach playback stopped: required gripper telemetry '
                       'lost or invalid'))
                return
        self._teach_last_valid_telemetry_ts = time.monotonic()

    def _finish_teach_loop(self) -> None:
        self._teach_playback_loop_index += 1
        if self._teach_playback_loop_index < self._teach_playback_loops:
            # Non-cyclic recordings must not jump end->start in one frame.
            self._begin_teach_loop()
            return
        self._complete_teach_playback()

    def _complete_teach_playback(self) -> None:
        token = self._teach_trajectory_token
        if token is not None and self.backend is not None:
            # Invalidate any late queued point but leave the final mode=0
            # target active, so closed-loop control continues holding it.
            self.backend.cancel_trajectory(token, hold_position=False)
        self._reset_teach_playback_state(clear_request=True)
        self._teach_set_active(False)
        self._teach_feedback('info', tr('Teach playback finished'))

    def _abort_teach_request(self, reason: str) -> None:
        """Give up on a queued (not yet running) playback and say why."""
        self._teach_playback_request = None
        self._teach_lock_req_ts = 0.0
        self._teach_enable_req_ts = 0.0
        self._teach_request_deadline = 0.0
        self._stop_teach_playback(clear_request=False)
        if reason:
            self._teach_feedback('warn', reason)

    def _stop_teach_playback(self, reason: str = '',
                             clear_request: bool = True,
                             hold_position: bool = True) -> None:
        was_pending = (bool(self._teach_playback_frames)
                       or self._teach_playback_request is not None
                       or self._teach_playback_phase != 'idle')
        if self._teach_playback_timer.isActive():
            self._teach_playback_timer.stop()
        token = self._teach_trajectory_token
        if token is not None and self.backend is not None:
            self.backend.cancel_trajectory(token, hold_position=hold_position)
        self._reset_teach_playback_state(clear_request=clear_request)
        self._teach_set_active(False)
        if reason and was_pending:
            self._teach_feedback('info', reason)

    def _reset_teach_playback_state(self, clear_request: bool) -> None:
        self._teach_playback_frames = []
        self._teach_playback_index = 0
        self._teach_playback_loop_index = 0
        self._teach_playback_phase = 'idle'
        self._teach_trajectory_token = None
        self._teach_trajectory_drain_deadline = 0.0
        self._teach_requires_gripper = False
        self._teach_last_valid_telemetry_ts = 0.0
        self._teach_preposition_target = {}
        self._teach_preposition_gripper = None
        self._teach_preposition_deadline = 0.0
        self._teach_preposition_stable = 0
        if clear_request:
            self._teach_playback_request = None

    def _build_statusbar(self) -> None:
        sb = self.statusBar()
        self.sb_backend = QLabel(tr('Backend: not connected'))
        self.sb_error = QLabel('')
        self.sb_error.setObjectName('statusError')
        limits = ', '.join('J%d[%.0f, %.0f]' % (j.joint_id, j.min_deg, j.max_deg)
                           for j in self.cfg.joints)
        self.sb_limits = QLabel(tr('Soft limits (°): %s | Step ≤%.0f° | Speed ≤%.0f r/min')
                                % (limits, self.cfg.safety.max_step_deg,
                                   self.cfg.safety.max_speed_rpm))
        self.sb_version = QLabel('v%s' % APP_VERSION)
        self.sb_version.setObjectName('dimLabel')
        sb.addWidget(self.sb_backend)
        sb.addWidget(self.sb_error, 1)
        sb.addPermanentWidget(self.sb_version)
        sb.addPermanentWidget(self.sb_limits)

    # ================= 后端生命周期 =================
    def _create_backend(self, kind: str) -> Optional[ArmBackend]:
        """Create the only backend exposed by the GUI: local CAN."""
        if kind != 'can':
            self.telemetry.append_log(
                'error', tr('Only local CAN connection is available'))
            self.sb_error.setText(
                tr('Only local CAN connection is available'))
            return None
        try:
            from ..core.can_backend import CanBackend
            backend: ArmBackend = CanBackend()
        except Exception as e:
            self.telemetry.append_log('error', tr('Failed to create %s backend: %s') % (kind, e))
            self.sb_error.setText(tr('Backend creation failed: %s') % e)
            return None
        # 灌入安全配置（软限位 + 运动参数上限）
        backend.safety = SafetyLimits(
            angle_limits=self.cfg.angle_limits,
            max_speed_rpm=self.cfg.safety.max_speed_rpm,
            max_accel=self.cfg.safety.max_accel,
            max_step_deg=self.cfg.safety.max_step_deg,
        )
        backend.state_updated.connect(self._on_state_updated)
        backend.connection_changed.connect(self._on_connection_changed)
        backend.log_message.connect(self.telemetry.append_log)
        backend.motion_enabled_changed.connect(self._on_motion_enabled)
        backend.joints_mode_changed.connect(self._on_joints_mode)
        backend.zero_setting_changed.connect(self._on_zero_setting_changed)
        return backend

    def _teardown_backend(self, final_close: bool = False) -> None:
        if self.backend is None:
            return
        self._request_safe_teach_state_before_teardown(final_close=final_close)
        backend, self.backend = self.backend, None
        # 先断开会改动 UI 状态的信号：shutdown 期间工作线程 emit 的排队事件
        # 不得在 teardown 之后送达（log_message 保留，shutdown 的日志仍可见）
        for sig, slot in ((backend.state_updated, self._on_state_updated),
                          (backend.connection_changed, self._on_connection_changed),
                          (backend.motion_enabled_changed, self._on_motion_enabled),
                          (backend.joints_mode_changed, self._on_joints_mode),
                          (backend.zero_setting_changed,
                           self._on_zero_setting_changed)):
            try:
                sig.disconnect(slot)
            except (RuntimeError, TypeError):
                pass
        try:
            backend.shutdown()
        except Exception as e:
            self.telemetry.append_log('warn', tr('Error shutting down old backend: %s') % e)
        backend.deleteLater()

    def _request_safe_teach_state_before_teardown(
            self, final_close: bool = False) -> None:
        """Submit teach hold/lock requests before invalidating the backend.

        Both backend calls are asynchronous.  Deliberately do not wait in the
        GUI thread; shutdown remains the single owner of worker termination.
        """
        backend = self.backend
        if backend is None:
            return
        teach = self._teach_dialog
        need_lock = self._joints_mode == 1
        if teach is not None:
            need_lock = (teach.prepare_for_shutdown(final_close=final_close)
                         or need_lock)
        # Active preposition/tracking owns a trajectory token, so this queues a
        # current-pose hold before teardown.  Pending-only requests are simply
        # cancelled because no trajectory command has started yet.
        self._stop_teach_playback(hold_position=True)
        if self._connected and need_lock:
            backend.set_joints_mode(2)

    def _on_connect_requested(self, kind: str, params: dict) -> None:
        self._teardown_backend()
        backend = self._create_backend(kind)
        if backend is None:
            self.conn_panel.set_state('disconnected', tr('Backend creation failed, see log for details'))
            return
        # 关节 ID/名字来自 AppConfig（连接面板不收集）：此处统一下发，
        # 保证后端轮询的关节与 UI 卡片/软限位使用同一套配置
        params = dict(params)
        params['joint_ids'] = self.cfg.joint_ids
        params['joint_names'] = self.cfg.joint_names
        params['gripper_id'] = self.cfg.gripper.motor_id
        params['gripper_speed_rpm'] = self.cfg.gripper.default_speed_rpm
        params['gripper_accel'] = self.cfg.gripper.default_accel
        self.backend = backend
        self._pending_kind = kind
        self.conn_panel.set_state('connecting', tr('Establishing connection…'), kind=kind)
        self.led.set_color(YELLOW)
        self.conn_lbl.setText(tr('Connecting…'))
        self.telemetry.append_log('info', tr('Connecting to %s backend…') % kind)
        backend.start(params)

    def _on_disconnect_requested(self) -> None:
        self.telemetry.append_log('info', tr('User requested disconnect'))
        self._teardown_backend()
        self._apply_disconnected(tr('Disconnected'))

    # ================= 后端信号槽（信号来自工作线程，Qt 自动排队） =================
    def _on_connection_changed(self, connected: bool, desc: str) -> None:
        # 跨线程信号按队列送达：断开/切换后端后，旧后端的残余事件仍可能到达，
        # 用 sender() 精确过滤（队列投递下 sender() 仍返回原发送者）
        if self.backend is None or self.sender() is not self.backend:
            return
        self._connected = connected
        if connected:
            kind = getattr(self, '_pending_kind', self.conn_panel.current_kind())
            self.led.set_color(GREEN)
            self.conn_lbl.setText(tr('Connected'))
            self.conn_panel.set_state('connected', desc, kind=kind)
            self.sb_backend.setText(tr('Backend: %s | %s') % (self.backend.name, desc))
            self.sb_error.setText('')
            # 连接成功 → 表单写回配置并持久化
            self.conn_panel.apply_to_config(self.cfg)
            self.cfg.last_backend = kind
            self._save_config()
            # 连接即控：自动开启运动使能（关节处于释放态时除外，锁定后自动恢复）
            if self._joints_mode != 1:
                self.backend.set_motion_enabled(True)
        else:
            self._apply_disconnected(desc)
        self._update_ui_state()

    def _apply_disconnected(self, desc: str) -> None:
        """统一处理断开后的 UI 复位：使能回 False、运动按钮禁用、卡片转灰。"""
        self._stop_teach_playback(tr('Teach playback stopped: disconnected'))
        self._connected = False
        self._last_state = None
        self._last_state_received_monotonic = 0.0
        self.led.set_color(FG_FAINT)
        self.conn_lbl.setText(tr('Not Connected'))
        self.conn_panel.set_state('disconnected', desc or tr('Not Connected'))
        self.sb_backend.setText(tr('Backend: not connected'))
        if desc:
            self.telemetry.append_log('info', tr('Connection status: %s') % desc)
        self._joints_mode = 2          # 下次连接按上电默认闭环显示，实际以读取为准
        self._apply_mode_style()
        self.joint_panel.clear_readings()
        self.telemetry.clear_values()
        set_zero = self._set_zero_dialog
        if set_zero is not None:
            set_zero.observe_state(None)
        self._update_ui_state()

    def _on_state_updated(self, state: ArmState) -> None:
        if self.backend is None or self.sender() is not self.backend:
            return                      # 已被 teardown 的旧后端残余信号
        self._last_state = state
        self._last_state_received_monotonic = time.monotonic()
        # Keep retrying queued teach playback start on each telemetry tick,
        # so async lock/enable transitions do not require an extra button click.
        if self._teach_playback_request is not None:
            self._try_start_teach_playback()
        self._observe_teach_preposition(state)
        self._observe_teach_tracking_health(state)
        self.joint_panel.update_state(state)
        self.telemetry.update_state(state)
        recorder = self._point_recorder_dialog
        if recorder is not None:
            recorder.observe_state(state)
        teach = self._teach_dialog
        if teach is not None:
            teach.observe_state(state)
        set_zero = self._set_zero_dialog
        if set_zero is not None:
            set_zero.observe_state(state)

    def _on_motion_enabled(self, enabled: bool) -> None:
        if self.backend is None or self.sender() is not self.backend:
            return                      # 已被 teardown 的旧后端残余信号
        self.telemetry.append_log('info', tr('Control resumed') if enabled
                                  else tr('Control paused (after E-stop/release, click "Lock Joints" to resume)'))
        if enabled:
            self._try_start_teach_playback()
        self._update_ui_state()

    def _on_joints_mode(self, mode: int) -> None:
        """关节模式变化：绿色高亮当前状态按钮（2=锁定绿 / 1=释放绿）。"""
        if self.backend is None or self.sender() is not self.backend:
            return
        self._joints_mode = int(mode)
        self._apply_mode_style()
        if self._joints_mode == 2:
            self._try_start_teach_playback()
        self._update_ui_state()

    def _on_zero_setting_changed(self, busy: bool, payload: object) -> None:
        """Forward asynchronous zero-write progress/result to its tool."""
        if self.backend is None or self.sender() is not self.backend:
            return
        dlg = self._set_zero_dialog
        if dlg is not None:
            dlg.show_zero_setting_state(bool(busy), payload)
        self._update_ui_state()

    def _apply_mode_style(self) -> None:
        self.btn_lock.setProperty('modeActive', self._joints_mode == 2)
        self.btn_release.setProperty('modeActive', self._joints_mode == 1)
        repolish(self.btn_lock)
        repolish(self.btn_release)

    # ================= 工具栏动作 =================
    def _motion_ready(self) -> bool:
        return (self.backend is not None and self._connected
                and self.backend.is_motion_enabled()
                and not self.backend.is_setting_zero()
                and not self._is_teach_busy())

    def _on_go_home(self) -> None:
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Homing rejected: not connected or motion not enabled'))
            return
        if not confirm(self, tr('Go to Zero'), tr(_ZERO_WARNING), estop_cb=self._on_estop):
            return
        targets = {jid: 0.0 for jid in self.cfg.joint_ids}
        # 回零自带确认框（上面刚确认过），大幅移动不再二次弹窗
        self._send_targets(targets, self.joint_panel.speed_rpm(),
                           self.joint_panel.accel(), tag='homing',
                           skip_large_confirm=True)

    def _zero_selection_snapshot(
            self, joint_ids: object
    ) -> Optional[tuple[list[int], Dict[int, float]]]:
        """Validate selected arm IDs against fresh online telemetry."""
        try:
            ids = list(joint_ids)  # type: ignore[arg-type]
        except TypeError:
            ids = []
        if (not ids
                or any(not isinstance(jid, int) or isinstance(jid, bool)
                       for jid in ids)
                or len(ids) != len(set(ids))):
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: select one or more valid arm joints'))
            return None
        unknown = [jid for jid in ids if jid not in self.cfg.joint_ids]
        if unknown:
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: unknown arm joint IDs %s') % unknown)
            return None
        if self._last_state is None:
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: fresh joint telemetry is required'))
            return None
        age = time.monotonic() - self._last_state_received_monotonic
        try:
            timestamp = float(self._last_state.timestamp)
        except (TypeError, ValueError):
            timestamp = math.nan
        state_age = time.time() - timestamp
        if (self._last_state_received_monotonic <= 0.0
                or age > self._SET_ZERO_TELEMETRY_TIMEOUT_S
                or not math.isfinite(timestamp)
                or not math.isfinite(state_age)
                or abs(state_age) > self._SET_ZERO_TELEMETRY_TIMEOUT_S):
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: joint telemetry is stale'))
            return None
        angles: Dict[int, float] = {}
        invalid = []
        moving = []
        for jid in ids:
            reading = self._last_state.joint(jid)
            if (reading is None or not reading.valid
                    or not math.isfinite(float(reading.angle_deg))):
                invalid.append(jid)
            else:
                angles[jid] = float(reading.angle_deg)
                speed = float(reading.speed_rpm)
                if (math.isfinite(speed)
                        and abs(speed) > self._SET_ZERO_STATIONARY_SPEED_RPM):
                    moving.append(jid)
        if invalid:
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: selected joints are offline or '
                           'invalid: %s') % invalid)
            return None
        if moving:
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: selected joints are moving: %s')
                % moving)
            return None
        return ids, angles

    def _on_set_selected_zero(self, joint_ids: object) -> None:
        """Confirm and persist zero for explicit J1-J4 IDs only."""
        backend = self.backend
        if backend is None or not self._connected:
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: not connected or motion not enabled'))
            return
        if self._is_teach_busy():
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: teach operation is active'))
            return
        if backend.is_setting_zero():
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: another zero-setting operation is active'))
            return
        if not backend.is_motion_enabled():
            self.telemetry.append_log(
                'warn', tr('Set zero rejected: not connected or motion not enabled'))
            return
        snapshot = self._zero_selection_snapshot(joint_ids)
        if snapshot is None:
            return
        ids, angles = snapshot
        details = '\n'.join(
            'ID %d: %.2f°' % (jid, angles[jid]) for jid in ids)
        if not confirm(
                self, tr('Set Selected to Zero'),
                tr(_SET_SELECTED_ZERO_WARNING) % details,
                estop_cb=self._on_estop):
            return

        # ``confirm`` runs a nested event loop.  Recheck connection, motion,
        # teach/busy gates and fresh per-ID telemetry before the Flash write.
        backend = self.backend
        if (backend is None or not self._connected
                or not backend.is_motion_enabled()
                or backend.is_setting_zero() or self._is_teach_busy()):
            self.telemetry.append_log(
                'warn', tr('Set zero cancelled: connection or motion enable '
                           'changed during confirmation'))
            return
        latest = self._zero_selection_snapshot(ids)
        if latest is None:
            self.telemetry.append_log(
                'warn', tr('Set zero cancelled: selected joint telemetry '
                           'changed during confirmation'))
            return
        backend.set_zero_here(ids)
        self.telemetry.append_log(
            'warn', tr('Requested persistent zero setting for arm joint IDs '
                       '%s (joints may briefly unload)') % ids)

    def _on_lock_joints(self) -> None:
        """锁定 = 闭环保持当前位置，安全操作无需确认。"""
        if self._is_teach_busy():
            self.telemetry.append_log(
                'warn', tr('Lock rejected: teach operation is active'))
            return
        self._set_joints_locked_direct()

    def _set_joints_locked_direct(self) -> None:
        """Lock joints immediately (no confirmation), for teach workflow."""
        if self.backend is None or not self._connected:
            self.telemetry.append_log('warn', tr('Lock rejected: not connected'))
            return
        if self.backend.is_setting_zero():
            self.telemetry.append_log(
                'warn', tr('Lock rejected: zero setting is active'))
            return
        self.backend.set_joints_mode(2)

    def _on_release_joints(self) -> None:
        """释放 = 待机卸载，机械臂会下垂 → 强确认（含急停按钮）。"""
        if self._is_teach_busy():
            self.telemetry.append_log(
                'warn', tr('Release rejected: teach operation is active'))
            return
        if self.backend is None or not self._connected:
            self.telemetry.append_log('warn', tr('Release rejected: not connected'))
            return
        if self.backend.is_setting_zero():
            self.telemetry.append_log(
                'warn', tr('Release rejected: zero setting is active'))
            return
        if not confirm(self, tr('Release Joints'), tr(_RELEASE_WARNING), estop_cb=self._on_estop):
            return
        if self.backend is None or not self._connected:
            self.telemetry.append_log('warn', tr('Release cancelled: connection lost during confirmation'))
            return
        if self.backend.is_setting_zero():
            self.telemetry.append_log(
                'warn', tr('Release cancelled: zero setting started during confirmation'))
            return
        self._stop_teach_playback(
            tr('Teach playback stopped: joints released'), hold_position=False)
        self.backend.set_joints_mode(1)

    def _set_joints_released_direct(self) -> None:
        """Release joints immediately (no confirmation), for teach workflow."""
        if self.backend is None or not self._connected:
            self.telemetry.append_log('warn', tr('Release rejected: not connected'))
            return
        if self.backend.is_setting_zero():
            self.telemetry.append_log(
                'warn', tr('Release rejected: zero setting is active'))
            return
        self._stop_teach_playback(
            tr('Teach playback stopped: joints released'), hold_position=False)
        self.backend.set_joints_mode(1)

    def _on_gripper_move(self, angle: float) -> None:
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Gripper command rejected: not connected or motion not enabled'))
            return
        self.backend.gripper_command(angle)

    def _on_gripper_zero(self) -> None:
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Gripper zero rejected: not connected or motion not enabled'))
            return
        if not confirm(self, tr('Set Gripper Closed Zero'),
                       tr('Set the CURRENT gripper position as the closed zero '
                          'point (0°)?\n\nMake sure the gripper is fully closed '
                          'first. This is persisted across power cycles and the '
                          'gripper motor will briefly unload during the write.'),
                       estop_cb=self._on_estop):
            return
        if not self._motion_ready():
            return
        self.backend.gripper_set_zero()

    def _on_estop(self) -> None:
        """急停：只要有后端就尽力发出（含「连接中」窗口期），无需确认。

        契约(backend_base)：estop() 在任何状态下都必须尽最大努力立即发出；
        两个后端未连接时调用都安全（只记日志），故不以 _connected 做闸门。
        """
        if self.backend is None:
            self.telemetry.append_log('warn', tr('E-stop not sent: no backend'))
            return
        self._stop_teach_playback(
            tr('Teach playback stopped: E-stop'), hold_position=False)
        self.backend.estop()
        self.telemetry.append_log('error', tr('E-stop command sent (broadcast); click "Lock Joints" to resume control'))
        self.sb_error.setText(tr('E-stop sent (click "Lock Joints" to resume control)'))

    # ================= 运动下发 =================
    def _on_move_requested(self, angles: Dict[int, float],
                           speed: float, accel: float) -> None:
        self._send_targets(dict(angles), speed, accel, tag='target')

    def _on_jog_requested(self, joint_id: int, delta: float) -> None:
        """相对步进点动：目标 = 当前实际角度 + 步进（不依赖目标输入框）。"""
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Jog rejected: not connected or motion not enabled'))
            return
        r = self._last_state.joint(joint_id) if self._last_state else None
        if r is None or not r.valid:
            self.telemetry.append_log('warn', tr('Jog rejected: joint %d has no valid reading') % joint_id)
            return
        self._send_targets({joint_id: r.angle_deg + delta},
                           self.joint_panel.speed_rpm(),
                           self.joint_panel.accel(), tag='jog')

    def _on_cartesian_requested(self, x: float, y: float, z: float) -> None:
        """XYZ 点位移动：Pitch 保持当前值，解析逆解 -> 电机角 -> 常规校验下发。"""
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Point move rejected: not connected or motion not enabled'))
            return
        from ..core import kinematics
        st = self._last_state
        motors = []
        for jid in (1, 2, 3, 4):
            r = st.joint(jid) if st else None
            if r is None or not r.valid:
                self.telemetry.append_log('warn', tr('Point move rejected: joints 1-4 must all be online'))
                return
            motors.append(r.angle_deg)
        j_cur = kinematics.motor_to_model(motors)
        pitch = j_cur[1] + j_cur[2] + j_cur[3] + kinematics.O3   # 保持当前俯仰
        sol = kinematics.ik(x, y, z, pitch, current=j_cur)
        if sol is None:
            msg = tr('No IK solution: point (%.1f, %.1f, %.1f, Pitch %.1f°) is out of reach or joint limits')
            self.telemetry.append_log('error', msg % (x, y, z, pitch))
            # 诊断提示：区分「超出可达球面」与「关节限位」
            import math as _m
            # TCP 目标按当前 Pitch 扣除夹爪长度得到腕点，再判断可达球面
            _p = _m.radians(pitch)
            _rw = _m.hypot(x, y) - kinematics.L_TOOL_MM * _m.cos(_p)
            _zw = z - kinematics.L_TOOL_MM * _m.sin(_p)
            r3 = _m.hypot(_rw, _zw)
            max_r = kinematics.A2 + kinematics.A3
            min_r = abs(kinematics.A3 - kinematics.A2)
            if r3 > max_r - 0.05:
                self.telemetry.append_log('warn', tr(
                    'Hint: after subtracting the gripper length at the current '
                    'pitch, the wrist target is %.1f mm from the shoulder, '
                    'at/beyond max reach %.1f mm — bring the point closer and retry')
                    % (r3, max_r))
            elif r3 < min_r + 0.05:
                self.telemetry.append_log('warn', tr(
                    'Hint: after subtracting the gripper length at the current '
                    'pitch, the wrist target is %.1f mm from the shoulder, '
                    'inside the minimum reach %.1f mm — move the point further out')
                    % (r3, min_r))
            self.sb_error.setText(tr('No IK solution'))
            return
        m_t = kinematics.model_to_motor(sol)
        self.telemetry.append_log('info', tr('IK solution (model deg): J1=%.2f J2=%.2f J3=%.2f J4=%.2f')
                                  % tuple(sol))
        self._send_targets({1: m_t[0], 2: m_t[1], 3: m_t[2], 4: m_t[3]},
                           self.joint_panel.speed_rpm(),
                           self.joint_panel.accel(), tag='point move')

    def _send_targets(self, angles: Dict[int, float], speed: float,
                      accel: float, tag: str = 'target',
                      skip_large_confirm: bool = False) -> None:
        """校验并下发目标。

        软限位超限：硬性拒绝。
        单步步长超限：弹确认框，用户确认后放行（skip_large_confirm=True 的
        调用方——如回零——已有自己的确认框，直接放行不再二次弹窗）。
        """
        if not self._motion_ready():
            self.telemetry.append_log('warn', tr('Command rejected: not connected or motion not enabled'))
            return
        allow_large = False
        ok, reason = self.backend.validate_targets(angles, self._last_state)
        if not ok:
            # 区分：软限位（硬拒绝） vs 仅步长超限（可确认放行）
            ok_soft, reason_soft = self.backend.validate_targets(
                angles, self._last_state, check_step=False)
            if not ok_soft:
                self.telemetry.append_log('error', tr('Command rejected: %s') % reason_soft)
                self.sb_error.setText(reason_soft)
                return
            if not skip_large_confirm:
                if not confirm(self, tr('Large Move'),
                               tr('The target is far from the current position:\n%s\n\n'
                                  'The arm will sweep a large arc — make sure the path '
                                  'is clear.\nExecute this large move anyway?') % reason,
                               estop_cb=self._on_estop):
                    self.telemetry.append_log('info', tr('Large move cancelled'))
                    return
                # 确认框期间可能发生断连，复查
                if not self._motion_ready():
                    self.telemetry.append_log('warn', tr('Command rejected: not connected or motion not enabled'))
                    return
            allow_large = True
        speed, accel = self.backend.clamp_motion_params(speed, accel)
        self.backend.command_angles(angles, speed, accel,
                                    allow_large_step=allow_large)
        desc = ', '.join('J%d→%.2f°' % (jid, a) for jid, a in sorted(angles.items()))
        self.telemetry.append_log('info', tr('Sent %s: %s (speed %.1f r/min, accel %.1f)')
                                  % (tr(tag), desc, speed, accel))
        self.sb_error.setText('')


    # ================= UI 状态联动 =================
    def _update_ui_state(self) -> None:
        connected = self._connected
        teach_busy = self._is_teach_busy()
        zero_busy = (self.backend is not None
                     and self.backend.is_setting_zero())
        enabled = self._motion_ready()
        self.btn_home.setEnabled(enabled)
        # Menu item is a navigation entry.  Keep it available while
        # disconnected/disabled (and during an active zero write) so the user
        # can inspect guidance and progress; only teach owns the whole UI.
        self._act_set_zero.setEnabled(not teach_busy)
        controls_idle = not teach_busy and not zero_busy
        self.btn_lock.setEnabled(connected and controls_idle)
        self.btn_release.setEnabled(connected and controls_idle)
        self._act_handeye.setEnabled(controls_idle)
        self._act_point_recorder.setEnabled(controls_idle)
        if self._act_teach is not None:
            self._act_teach.setEnabled(controls_idle)
        # 急停：仅未连接时禁用（样式保持红色，见 QSS #estopButton:disabled）
        self.btn_estop.setEnabled(connected)
        self.joint_panel.set_connected(connected)
        self.joint_panel.set_motion_allowed(enabled)
        recorder = self._point_recorder_dialog
        if recorder is not None:
            recorder.update_control_state()
            recorder.setEnabled(controls_idle)
        calib = self._calib_dialog
        if calib is not None:
            calib.setEnabled(controls_idle)
        teach = self._teach_dialog
        if teach is not None:
            teach.update_control_state()
            teach.setEnabled(not zero_busy)
        set_zero = self._set_zero_dialog
        if set_zero is not None:
            set_zero.update_control_state()

    # ================= 持久化 =================
    def _save_config(self) -> None:
        try:
            # safety/joints 在 GUI 中不可编辑：保存前以磁盘为准，
            # 保留用户在 config.json 里的手工修改（如 max_step_deg），
            # 避免退出时被内存里的旧值覆盖
            disk = (AppConfig.load(self._config_path) if self._config_path
                    else AppConfig.load())
            self.cfg.safety = disk.safety
            self.cfg.joints = disk.joints
            if self._config_path:
                self.cfg.save(self._config_path)
            else:
                self.cfg.save()
        except Exception as e:
            self.telemetry.append_log('warn', tr('Failed to save config: %s') % e)

    def _settings(self) -> QSettings:
        return QSettings('DrEmpower', 'ArmGUI')

    def _restore_geometry(self) -> None:
        """Restore a useful screen and constrain the normal window rectangle.

        ``QMainWindow.restoreGeometry`` can faithfully restore a rectangle
        which belonged to a monitor that has since been unplugged, or whose
        work area became smaller after a DPI/taskbar change.  Correct the
        normal rectangle first so the later :meth:`showMaximized` call knows
        which *current* monitor owns the window instead of preserving an
        oversized/off-screen rectangle.
        """
        # 布局固定：只恢复窗口位置/尺寸，不恢复 dock 状态（避免旧会话的浮动布局）
        geo = self._settings().value('geometry')
        if geo is not None:
            self.restoreGeometry(geo)

        # saveGeometry() also remembers maximised/minimised/full-screen state.
        # Capture its normal rectangle, then temporarily return to a normal
        # state: some window managers ignore setGeometry() while a maximised
        # state is still pending, even before the widget becomes visible.
        rect = self.normalGeometry()
        special_states = (Qt.WindowState.WindowMinimized |
                          Qt.WindowState.WindowMaximized |
                          Qt.WindowState.WindowFullScreen)
        normal_state = self.windowState() & ~special_states
        self.setWindowState(normal_state)

        screens = QGuiApplication.screens()
        if screens:
            # Prefer the screen containing most of the restored window.  If
            # the saved monitor disappeared, fall back to the primary screen.
            target = None
            best_area = 0
            for screen in screens:
                overlap = rect.intersected(screen.availableGeometry())
                area = max(0, overlap.width()) * max(0, overlap.height())
                if area > best_area:
                    target = screen
                    best_area = area
            if target is None:
                target = QGuiApplication.primaryScreen() or screens[0]

            available = target.availableGeometry()
            width = min(max(1, rect.width()), available.width())
            height = min(max(1, rect.height()), available.height())
            max_x = available.right() - width + 1
            max_y = available.bottom() - height + 1
            x = min(max(rect.x(), available.left()), max_x)
            y = min(max(rect.y(), available.top()), max_y)
            self.setGeometry(x, y, width, height)

    def closeEvent(self, event) -> None:  # noqa: N802（Qt 命名）
        st = self._settings()
        st.setValue('geometry', self.saveGeometry())
        self.conn_panel.apply_to_config(self.cfg)
        self._save_config()
        self.camera_view.stop()
        self._teardown_backend(final_close=True)
        super().closeEvent(event)


# ================= i18n 词表（英文原文 -> 改写前中文，逐字保留） =================
register({
    'jog': '点动',
    'point move': '点位移动',
    'Gripper command rejected: not connected or motion not enabled':
        '夹爪命令被拒绝：未连接或未使能运动',
    'Gripper zero rejected: not connected or motion not enabled':
        '夹爪置零被拒绝：未连接或未使能运动',
    'Set Gripper Closed Zero': '设置夹爪闭合零点',
    'Set the CURRENT gripper position as the closed zero point (0°)?\n\n'
    'Make sure the gripper is fully closed first. This is persisted across '
    'power cycles and the gripper motor will briefly unload during the write.':
        '把夹爪【当前】位置设为闭合零点（0°）吗？\n\n请先确认夹爪已完全闭合。'
        '该设置掉电保存，写入过程中夹爪电机会短暂卸载。',
    'Large Move': '大幅移动',
    'The target is far from the current position:\n%s\n\n'
    'The arm will sweep a large arc — make sure the path is clear.\n'
    'Execute this large move anyway?':
        '目标距当前位置较远：\n%s\n\n机械臂将扫过较大弧线——请确认路径上无障碍物。\n'
        '确认执行这次大幅移动吗？',
    'Large move cancelled': '大幅移动已取消',
    'Jog rejected: not connected or motion not enabled':
        '点动被拒绝：未连接或未使能运动',
    'Jog rejected: joint %d has no valid reading':
        '点动被拒绝：关节 %d 无有效读数',
    'Point move rejected: not connected or motion not enabled':
        '点位移动被拒绝：未连接或未使能运动',
    'Point move rejected: joints 1-4 must all be online':
        '点位移动被拒绝：关节 1~4 必须全部在线',
    'No IK solution: point (%.1f, %.1f, %.1f, Pitch %.1f°) is out of reach or joint limits':
        '逆解无解：点位 (%.1f, %.1f, %.1f, Pitch %.1f°) 超出可达范围或关节限位',
    'No IK solution': '逆解无解',
    'Hint: after subtracting the gripper length at the current pitch, the '
    'wrist target is %.1f mm from the shoulder, at/beyond max reach %.1f mm '
    '— bring the point closer and retry':
        '提示：按当前 Pitch 扣除夹爪长度后，腕点距肩部 %.1f mm，达到/超出最大可达 '
        '%.1f mm——请把点位收近一些再试',
    'Hint: after subtracting the gripper length at the current pitch, the '
    'wrist target is %.1f mm from the shoulder, inside the minimum reach '
    '%.1f mm — move the point further out':
        '提示：按当前 Pitch 扣除夹爪长度后，腕点距肩部 %.1f mm，小于最小可达 '
        '%.1f mm——请把点位放远一些',
    'IK solution (model deg): J1=%.2f J2=%.2f J3=%.2f J4=%.2f':
        '逆解结果（模型角°）：J1=%.2f J2=%.2f J3=%.2f J4=%.2f',
    'DrEmpower Arm Console': 'DrEmpower 机械臂控制台',
    'Console started (connect to take control; Esc E-stop is always '
    'available; after E-stop/release, click "Lock Joints" to resume '
    'control)':
        '控制台已启动（连接后即可控制；急停 Esc 随时可用，'
        '急停/释放后点「锁定关节」恢复控制）',
    'Main Toolbar': '主工具栏',
    'Not Connected': '未连接',
    'Go to Zero': '回到零位',
    'Move all joints to 0° (motion enable required)':
        '全部关节移动到 0°（需使能运动）',
    'Set Zero…': '设置零点…',
    'Open the joint zero-setting tool': '打开关节零点设置功能',
    'Lock Joints': '锁定关节',
    'All joints enter closed-loop mode: hold current position (joints must be locked before motion)':
        '全部关节进入闭环模式：保持当前位置（运动前必须处于锁定状态）',
    'Release Joints': '释放关节',
    'All joints enter standby mode: unloaded for manual positioning (the arm will sag; confirmation required)':
        '全部关节进入待机模式：卸载可手动摆位（机械臂会下垂，需确认）',
    'E-STOP (Esc)': '急  停 (Esc)',
    'Broadcast E-stop immediately (no confirmation; unavailable when not connected)':
        '立即广播急停（无需确认；未连接时不可用）',
    'Connection': '连接',
    'Telemetry': '遥测',
    'Language': '语言',
    'Menu': '菜单',
    'Calibration': '标定',
    'Hand-Eye Calibration…': '手眼标定…',
    'Manual Point Recorder…': '手动点位记录器…',
    'Teach Playback…': '示教回放…',
    'Switch Language': '切换语言',
    'The new language takes effect after restart. Restart now?':
        '界面语言将在重启后生效，现在重启应用吗？',
    'Backend: not connected': '后端: 未连接',
    'Soft limits (°): %s | Step ≤%.0f° | Speed ≤%.0f r/min':
        '软限位(°): %s | 单步≤%.0f° | 速度≤%.0f r/min',
    'Failed to create %s backend: %s': '创建 %s 后端失败: %s',
    'Backend creation failed: %s': '后端创建失败: %s',
    'Only local CAN connection is available': '仅提供本机 CAN 连接',
    'Error shutting down old backend: %s': '关闭旧后端异常: %s',
    'Backend creation failed, see log for details': '后端创建失败，详见日志',
    'Establishing connection…': '正在连接…',
    'Connecting…': '连接中…',
    'Connecting to %s backend…': '开始连接 %s 后端…',
    'User requested disconnect': '用户请求断开连接',
    'Disconnected': '已断开',
    'Connected': '已连接',
    'Backend: %s | %s': '后端: %s | %s',
    'Connection status: %s': '连接状态: %s',
    'Control resumed': '控制已恢复',
    'Control paused (after E-stop/release, click "Lock Joints" to resume)':
        '控制已暂停（急停/释放后点「锁定关节」恢复）',
    'Homing rejected: not connected or motion not enabled':
        '回零被拒绝：未连接或未使能运动',
    'Set zero rejected: not connected or motion not enabled':
        '设零点被拒绝：未连接或未使能运动',
    'Set zero rejected: select one or more valid arm joints':
        '设零点被拒绝：请选择一个或多个有效机械臂关节',
    'Set zero rejected: unknown arm joint IDs %s':
        '设零点被拒绝：未知机械臂关节 ID %s',
    'Set zero rejected: fresh joint telemetry is required':
        '设零点被拒绝：需要最新关节遥测数据',
    'Set zero rejected: joint telemetry is stale':
        '设零点被拒绝：关节遥测数据已过期',
    'Set zero rejected: selected joints are offline or invalid: %s':
        '设零点被拒绝：所选关节离线或数据无效：%s',
    'Set zero rejected: selected joints are moving: %s':
        '设零点被拒绝：所选关节仍在运动：%s',
    'Set zero rejected: teach operation is active':
        '设零点被拒绝：示教操作正在进行',
    'Set zero rejected: another zero-setting operation is active':
        '设零点被拒绝：另一个设零操作正在进行',
    'Set zero cancelled: connection or motion enable changed during confirmation':
        '设零点已取消：确认期间连接或运动使能状态发生变化',
    'Set zero cancelled: selected joint telemetry changed during confirmation':
        '设零点已取消：确认期间所选关节遥测状态发生变化',
    'Requested persistent zero setting for arm joint IDs %s (joints may briefly unload)':
        '已请求为机械臂关节 ID %s 持久设置零点（关节可能短暂卸载）',
    'Lock rejected: not connected': '锁定被拒绝：未连接',
    'Lock rejected: teach operation is active': '锁定被拒绝：示教操作正在进行',
    'Lock rejected: zero setting is active': '锁定被拒绝：正在设置零点',
    'Release rejected: not connected': '释放被拒绝：未连接',
    'Release rejected: teach operation is active': '释放被拒绝：示教操作正在进行',
    'Release rejected: zero setting is active': '释放被拒绝：正在设置零点',
    'Release cancelled: connection lost during confirmation':
        '释放已取消：确认期间连接已断开',
    'Release cancelled: zero setting started during confirmation':
        '释放已取消：确认期间开始了零点设置',
    'E-stop not sent: no backend': '急停未发送：无后端',
    'E-stop command sent (broadcast); click "Lock Joints" to resume control':
        '已发送急停指令（广播）；点「锁定关节」恢复控制',
    'E-stop sent (click "Lock Joints" to resume control)':
        '已发送急停（点「锁定关节」恢复控制）',
    'Command rejected: not connected or motion not enabled':
        '命令被拒绝：未连接或未使能运动',
    'Command rejected: %s': '命令被拒绝：%s',
    'Sent %s: %s (speed %.1f r/min, accel %.1f)':
        '下发%s: %s (速度 %.1f r/min, 加速度 %.1f)',
    'homing': '回零',
    'target': '目标',
    'Failed to save config: %s': '配置保存失败: %s',
    'Teach playback rejected: not connected': '示教回放被拒绝：未连接',
    'Teach playback rejected: not enough samples': '示教回放被拒绝：样本不足',
    'Teach playback requires locked joints': '示教回放需要先锁定关节',
    'Teach playback requested lock; retry playback after lock is active':
        '示教回放已请求锁定；锁定生效后请再次点击回放',
    'Teach playback queued: waiting for lock and motion enable':
        '示教回放已排队：等待锁定与运动使能就绪',
    'Teach playback waiting: requested joint lock':
        '示教回放等待中：已请求锁定关节',
    'Teach playback waiting: requested motion enable':
        '示教回放等待中：已请求开启运动使能',
    'Teach playback started: %d samples × %d loop(s)': '示教回放已开始：%d 个样本 × %d 轮',
    'Teach playback rejected: backend has no streamed trajectory support':
        '示教回放被拒绝：当前后端不支持流式轨迹',
    'Teach loop %d/%d: moving to the first sample':
        '示教回放第 %d/%d 轮：正在以梯形轨迹移动到首个样本',
    'Teach loop %d/%d: first sample reached; tracking started':
        '示教回放第 %d/%d 轮：已到达首个样本，开始流式轨迹跟踪',
    'Teach playback cancelled: first sample was not reached in time':
        '示教回放已取消：未能在时限内到达首个样本',
    'Teach playback stopped: trajectory session lost':
        '示教回放已停止：轨迹会话已失效',
    'Teach playback stopped: final trajectory point did not drain':
        '示教回放已停止：末个轨迹点未能及时发送完成',
    'Teach playback stopped: required telemetry timed out':
        '示教回放已停止：必需遥测已超时',
    'Teach playback stopped: required joint J%d telemetry lost or invalid':
        '示教回放已停止：必需关节 J%d 遥测丢失或无效',
    'Teach playback stopped: required gripper telemetry lost or invalid':
        '示教回放已停止：必需夹爪遥测丢失或无效',
    'Teach playback stopped: joints released': '示教回放已停止：关节已释放',
    'Teach playback stopped: disconnected': '示教回放已停止：连接断开',
    'Teach playback finished': '示教回放完成',
    'Teach playback stopped: motion not enabled': '示教回放已停止：运动未使能',
    'Teach playback stopped: E-stop': '示教回放已停止：已急停',
    'Teach playback stopped by user': '示教回放已由用户停止',
    'Teach playback is disabled (set ui.enable_teach_playback=true in config.json to re-enable)':
        '示教回放已停用（在 config.json 中把 ui.enable_teach_playback 改为 true 可重新启用）',
    'Teach playback rejected: sample %d has no joint data':
        '示教回放被拒绝：第 %d 个样本没有关节数据',
    'Teach playback rejected: sample %d must contain joints %s':
        '示教回放被拒绝：第 %d 个样本必须包含关节 %s',
    'Teach playback rejected: sample %d has an invalid timestamp':
        '示教回放被拒绝：第 %d 个样本时间戳无效',
    'Teach playback rejected: sample %d has a non-finite angle':
        '示教回放被拒绝：第 %d 个样本包含无效关节角度',
    'Teach playback rejected: sample %d has a non-finite gripper angle':
        '示教回放被拒绝：第 %d 个样本包含无效夹爪角度',
    'Teach playback rejected: unknown joint ID %s': '示教回放被拒绝：未知关节 ID %s',
    'Teach playback rejected: sample %d J%d %.2f° is outside the soft limit [%.1f, %.1f]°':
        '示教回放被拒绝：第 %d 个样本 J%d %.2f° 超出软限位 [%.1f, %.1f]°',
    'Teach playback cancelled: joints did not reach closed loop within %.0f s '
    '(check the drive error log, then lock manually)':
        '示教回放已取消：%.0f 秒内关节未进入闭环（请查看驱动器错误日志，'
        '然后手动锁定关节）',
    'Teach playback cancelled: motion enable was not granted within %.0f s '
    '(check the drive error log)':
        '示教回放已取消：%.0f 秒内运动使能未通过（请查看驱动器错误日志）',
    _ZERO_WARNING:
        '即将命令全部关节回到 0° 位置。\n\n'
        '机械臂将以当前设定的速度/加速度执行梯形轨迹运动，\n'
        '请确认运动路径上无人员和障碍物。\n\n'
        '确定执行回零吗？',
    _SET_SELECTED_ZERO_WARNING:
        '即将把以下机械臂关节的当前位置持久设置为 0°：\n%s\n\n'
        '写入驱动器配置时，每个所选关节都可能短暂失去保持力矩。'
        '请可靠支撑机械臂。\n\n'
        '此操作会改变关节坐标基准；已有示教录制、标定数据和保存点位可能无法再'
        '正确复现。\n\n'
        '确定设置所选关节零点吗？',
    _RELEASE_WARNING:
        '即将释放全部关节（切换到待机模式）！\n\n'
        '关节将失去保持力矩，机械臂会在重力作用下下垂——\n'
        '请先扶稳机械臂，或确认其处于稳定/支撑姿态。\n'
        '释放后可手动摆位；需点击「锁定关节」恢复闭环后才能再运动。\n\n'
        '确定释放吗？',
})
