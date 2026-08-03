# -*- coding: utf-8 -*-
"""右侧「遥测」面板：母线电压/电流、通信延迟、刷新率、在线关节数 + 日志台。

日志台：QPlainTextEdit 只读、按等级着色、上限 2000 行、
自动滚动可通过复选框暂停。
"""
from __future__ import annotations

import html
import math
import time
from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPlainTextEdit,
    QPushButton, QVBoxLayout, QWidget,
)

from ..core import kinematics
from ..core.state import ArmState
from ..i18n import register, tr
from .style import FG_DIM, FG_TEXT, RED, YELLOW, mono_font

# FK 需要的关节集合：标定模型是固定的 4 轴（CAN ID 1~4 电机原始角）
_FK_IDS = (1, 2, 3, 4)

_MAX_LOG_LINES = 2000

_LEVEL_COLOR = {
    'info': FG_TEXT,
    'warn': YELLOW,
    'error': RED,
}


class _Tile(QFrame):
    """遥测数字块：标题小字 + 大号等宽数值 + 单位。"""

    def __init__(self, title: str, unit: str = '',
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setObjectName('telemetryTile')
        t = QLabel(title)
        t.setObjectName('tileTitle')
        self.value_lbl = QLabel('--')
        self.value_lbl.setObjectName('tileValue')
        self.value_lbl.setFont(mono_font(15, bold=True))
        self.value_lbl.setAlignment(Qt.AlignmentFlag.AlignRight |
                                    Qt.AlignmentFlag.AlignVCenter)
        u = QLabel(unit)
        u.setObjectName('tileUnit')
        row = QHBoxLayout()
        row.setSpacing(3)
        row.addStretch(1)
        row.addWidget(self.value_lbl)
        row.addWidget(u, 0, Qt.AlignmentFlag.AlignBottom)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        lay.addWidget(t)
        lay.addLayout(row)

    def set_value(self, text: str) -> None:
        self.value_lbl.setText(text)


class TelemetryPanel(QWidget):
    """遥测 Dock 内容。update_state() 由 state_updated 信号驱动。"""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._last_recv: Optional[float] = None   # 上次收到状态的时刻（monotonic）
        self._rate_hz = 0.0
        self._lat_ema: Optional[float] = None     # 延迟滑动平均（显示防跳动）

        # ---- 数字块 ----
        self.tile_volt = _Tile(tr('Bus Voltage'), 'V')
        self.tile_curr = _Tile(tr('Bus Current'), 'A')
        self.tile_lat = _Tile(tr('Comm Latency'), 'ms')
        self.tile_rate = _Tile(tr('State Refresh Rate'), 'Hz')
        self.tile_online = _Tile(tr('Online Joints'), '')
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.addWidget(self.tile_volt, 0, 0)
        grid.addWidget(self.tile_curr, 0, 1)
        grid.addWidget(self.tile_lat, 1, 0)
        grid.addWidget(self.tile_rate, 1, 1)
        grid.addWidget(self.tile_online, 2, 0, 1, 2)

        # ---- 末端位姿（标定 FK，2026-06-24 标定 + 2026-07-18 置零映射）----
        fk_title = QLabel(tr('TCP Pose · Calibrated FK · model frame (shoulder origin)'))
        fk_title.setObjectName('dimLabel')
        self.tile_x = _Tile('X', 'mm')
        self.tile_y = _Tile('Y', 'mm')
        self.tile_z = _Tile('Z', 'mm')
        self.tile_pitch = _Tile('Pitch', '°')
        # 世界系高度（桌面=0；Z0 已于 2026-07-18 标定）
        self.tile_zw = _Tile(tr('Height above table'), 'mm')
        fk_grid = QGridLayout()
        fk_grid.setSpacing(6)
        fk_grid.addWidget(self.tile_x, 0, 0)
        fk_grid.addWidget(self.tile_y, 0, 1)
        fk_grid.addWidget(self.tile_z, 1, 0)
        fk_grid.addWidget(self.tile_pitch, 1, 1)
        fk_grid.addWidget(self.tile_zw, 2, 0, 1, 2)

        # ---- 日志台 ----
        log_title = QLabel(tr('Log'))
        log_title.setObjectName('dimLabel')
        self.chk_scroll = QCheckBox(tr('Auto-scroll'))
        self.chk_scroll.setChecked(True)
        btn_clear = QPushButton(tr('Clear'))
        btn_clear.setMinimumWidth(48)
        log_bar = QHBoxLayout()
        log_bar.setSpacing(6)
        log_bar.addWidget(log_title)
        log_bar.addStretch(1)
        log_bar.addWidget(self.chk_scroll)
        log_bar.addWidget(btn_clear)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(_MAX_LOG_LINES)
        self.log.setFont(mono_font(8))
        # 固定布局下日志面板宽度有限：按控件宽度自动换行，长日志不再被截断
        self.log.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        btn_clear.clicked.connect(self.log.clear)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
        lay.addLayout(grid, 0)
        lay.addWidget(fk_title, 0)
        lay.addLayout(fk_grid, 0)
        lay.addLayout(log_bar, 0)
        lay.addWidget(self.log, 1)

    # ================= 遥测刷新 =================
    def update_state(self, state: ArmState) -> None:
        now = time.monotonic()
        if self._last_recv is not None:
            dt = now - self._last_recv
            if dt > 1e-4:
                inst = 1.0 / dt
                # 指数滑动平均，读数稳定
                self._rate_hz = (inst if self._rate_hz <= 0.0
                                 else self._rate_hz * 0.8 + inst * 0.2)
        self._last_recv = now

        self.tile_volt.set_value(self._fmt(state.bus_voltage, '%.1f'))
        self.tile_curr.set_value(self._fmt(state.bus_current, '%.2f'))
        # 延迟做指数滑动平均：单次抖动（USB 批次交付）不至于让数字乱跳
        if not math.isnan(state.latency_ms):
            self._lat_ema = (state.latency_ms if self._lat_ema is None
                             else self._lat_ema * 0.7 + state.latency_ms * 0.3)
        self.tile_lat.set_value('%.0f' % self._lat_ema
                                if self._lat_ema is not None else '--')
        self.tile_rate.set_value('%.1f' % self._rate_hz if self._rate_hz > 0 else '--')
        self.tile_online.set_value('%d / %d' % (state.online_count, len(state.joints)))
        self._update_fk(state)

    def _update_fk(self, state: ArmState) -> None:
        """标定 FK：需要 ID 1~4 全部在线（电机原始角），否则显示 --。"""
        motors = []
        for jid in _FK_IDS:
            j = state.joint(jid)
            if j is None or not j.valid:
                for tile in (self.tile_x, self.tile_y, self.tile_z,
                             self.tile_pitch, self.tile_zw):
                    tile.set_value('--')
                return
            motors.append(j.angle_deg)
        x, y, z, pitch = kinematics.fk_from_motor(motors)
        self.tile_x.set_value('%+.1f' % x)
        self.tile_y.set_value('%+.1f' % y)
        self.tile_z.set_value('%+.1f' % z)
        self.tile_pitch.set_value('%+.1f' % pitch)
        zw = kinematics.world_z(z)
        self.tile_zw.set_value('%+.1f' % zw if zw is not None else '--')

    def clear_values(self) -> None:
        """断开后遥测数字复位。"""
        self._last_recv = None
        self._rate_hz = 0.0
        self._lat_ema = None
        for tile in (self.tile_volt, self.tile_curr, self.tile_lat,
                     self.tile_rate, self.tile_online,
                     self.tile_x, self.tile_y, self.tile_z, self.tile_pitch,
                     self.tile_zw):
            tile.set_value('--')

    @staticmethod
    def _fmt(value: float, fmt: str) -> str:
        return '--' if value is None or math.isnan(value) else fmt % value

    # ================= 日志台 =================
    def append_log(self, level: str, text: str) -> None:
        """接 backend.log_message(level, text)；level ∈ info|warn|error。"""
        color = _LEVEL_COLOR.get(level, FG_TEXT)
        ts = time.strftime('%H:%M:%S')
        line = ('<span style="color:%s">%s</span> '
                '<span style="color:%s">[%s] %s</span>'
                % (FG_DIM, ts, color, level.upper(), html.escape(str(text))))
        self.log.appendHtml(line)
        if self.chk_scroll.isChecked():
            sb = self.log.verticalScrollBar()
            sb.setValue(sb.maximum())


register({
    'Height above table': '离桌面高度',
    'Bus Voltage': '母线电压',
    'Bus Current': '母线电流',
    'Comm Latency': '通信延迟',
    'State Refresh Rate': '状态刷新率',
    'Online Joints': '在线关节',
    'TCP Pose · Calibrated FK · model frame (shoulder origin)':
        '末端位姿 · 标定FK · 模型系(肩部原点)',
    'Log': '日志',
    'Auto-scroll': '自动滚动',
    'Clear': '清空',
})
