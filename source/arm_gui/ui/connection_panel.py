# -*- coding: utf-8 -*-
"""CAN direct-connection panel."""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..core.config import AppConfig
from ..i18n import register, tr
from .style import mono_font


# USB-CAN module VID:PID; used to select the adapter when present.
_USBCAN_VIDPID = '28E9:018A'
_BAUD_CHOICES = [
    '115200', '230400', '460800', '921600', '1000000', '2000000'
]


class ConnectionPanel(QWidget):
    """Local CAN connection controls and connection details."""

    connect_requested = Signal(str, dict)  # kind is always ``can``
    disconnect_requested = Signal()

    def __init__(self, cfg: AppConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.cfg = cfg
        self._state = 'disconnected'
        self._active_kind = ''

        self.tabs = QTabWidget(self)
        self.tabs.addTab(self._build_can_tab(), tr('CAN Direct'))

        self.detail = QPlainTextEdit(self)
        self.detail.setReadOnly(True)
        self.detail.setFixedHeight(64)
        self.detail.setFont(mono_font(8))
        self.detail.setPlaceholderText(
            tr('Connection details will be shown here'))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)
        layout.addWidget(self.tabs, 1)
        layout.addWidget(self.detail, 0)

        self.refresh_ports()

    def _build_can_tab(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        form.setContentsMargins(8, 10, 8, 8)
        form.setSpacing(8)

        self.can_port = QComboBox()
        self.can_port.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.can_port.setMinimumContentsLength(8)
        refresh = QPushButton(tr('Refresh'))
        refresh.clicked.connect(self.refresh_ports)
        row = QHBoxLayout()
        row.setSpacing(4)
        row.addWidget(self.can_port, 1)
        row.addWidget(refresh, 0)
        form.addRow(tr('COM Port'), row)

        self.can_baud = QComboBox()
        self.can_baud.setEditable(True)
        self.can_baud.addItems(_BAUD_CHOICES)
        self.can_baud.setCurrentText(str(self.cfg.can.baud))
        form.addRow(tr('Baud Rate'), self.can_baud)

        self.can_poll = QDoubleSpinBox()
        self.can_poll.setRange(1.0, 100.0)
        self.can_poll.setDecimals(1)
        self.can_poll.setSuffix(' Hz')
        self.can_poll.setValue(self.cfg.can.poll_hz)
        form.addRow(tr('Poll Rate'), self.can_poll)

        self.btn_can = QPushButton(tr('Connect'))
        self.btn_can.setObjectName('primaryButton')
        self.btn_can.clicked.connect(lambda: self._on_button('can'))
        form.addRow('', self.btn_can)
        return page

    def refresh_ports(self) -> None:
        """Enumerate local serial ports and prefer the USB-CAN adapter."""
        self.can_port.clear()
        try:
            from serial.tools import list_ports
            ports = sorted(list_ports.comports(), key=lambda p: p.device)
        except Exception:
            self.can_port.addItem(tr('(pyserial not installed)'), '')
            return
        if not ports:
            self.can_port.addItem(tr('(no serial ports detected)'), '')
            return
        pick = -1
        for index, port in enumerate(ports):
            description = (port.description or '').strip()
            self.can_port.addItem(
                '%s  %s' % (port.device, description), port.device)
            hwid = (port.hwid or '').upper()
            if _USBCAN_VIDPID in hwid:
                pick = index
            elif pick < 0 and port.device == self.cfg.can.port:
                pick = index
        self.can_port.setCurrentIndex(max(pick, 0))

    def _on_button(self, _kind: str = 'can') -> None:
        if self._state in ('connected', 'connecting'):
            self.disconnect_requested.emit()
            return
        self.connect_requested.emit('can', self.can_params())

    def can_params(self) -> dict:
        try:
            baud = int(self.can_baud.currentText())
        except ValueError:
            baud = 115200
        return {
            'port': self.can_port.currentData() or '',
            'baud': baud,
            'poll_hz': float(self.can_poll.value()),
        }

    def current_kind(self) -> str:
        return 'can'

    def set_state(self, state: str, desc: str = '', kind: str = '') -> None:
        self._state = state
        if kind:
            self._active_kind = 'can'
        if desc:
            self.detail.setPlainText(desc)

        connected_like = state in ('connecting', 'connected')
        for widget in (self.can_port, self.can_baud, self.can_poll):
            widget.setEnabled(not connected_like)

        if not connected_like:
            self.btn_can.setText(tr('Connect'))
            self.btn_can.setEnabled(True)
        else:
            self.btn_can.setText(
                tr('Disconnect') if state == 'connected'
                else tr('Connecting…(click to cancel)'))
            self.btn_can.setEnabled(True)

        if state == 'disconnected':
            self._active_kind = ''

    def apply_to_config(self, cfg: AppConfig) -> None:
        params = self.can_params()
        if params['port']:
            cfg.can.port = params['port']
        cfg.can.baud = params['baud']
        cfg.can.poll_hz = params['poll_hz']
        cfg.last_backend = 'can'


register({
    'CAN Direct': 'CAN 直连',
    'Connection details will be shown here': '连接详情将显示在这里',
    'Refresh': '刷新',
    'COM Port': 'COM 端口',
    'Baud Rate': '波特率',
    'Poll Rate': '轮询频率',
    'Connect': '连接',
    '(pyserial not installed)': '（未安装 pyserial）',
    '(no serial ports detected)': '（未检测到串口）',
    'Disconnect': '断开',
    'Connecting…(click to cancel)': '连接中…（点击取消）',
})
