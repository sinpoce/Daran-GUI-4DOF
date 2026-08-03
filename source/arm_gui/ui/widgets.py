# -*- coding: utf-8 -*-
"""可复用基础控件：状态灯、迷你火花线、MJPEG 视图、确认对话框。

所有控件均为纯 UI（不触碰任何硬件/网络，MJPEG 除外——其网络访问
完全由 QNetworkAccessManager 异步驱动，不阻塞 GUI 线程）。
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Callable, Deque, Optional, Tuple

from PySide6.QtCore import QPointF, QSize, Qt, QUrl
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import QLabel, QMessageBox, QSizePolicy, QVBoxLayout, QWidget

from ..i18n import tr, register
from .style import ACCENT, BG_INPUT, BORDER, FG_FAINT


class StatusLed(QWidget):
    """圆形状态指示灯。set_color() 换色（用 style.py 中的颜色常量）。"""

    def __init__(self, color: str = FG_FAINT, diameter: int = 12,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._color = QColor(color)
        self._d = diameter
        self.setFixedSize(diameter + 4, diameter + 4)

    def set_color(self, color: str) -> None:
        c = QColor(color)
        if c != self._color:
            self._color = c
            self.update()

    def paintEvent(self, event) -> None:  # noqa: N802（Qt 命名）
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        # 外圈暗边 + 实心灯芯
        p.setPen(QPen(self._color.darker(160), 1))
        p.setBrush(self._color)
        r = self._d
        p.drawEllipse(2, 2, r, r)
        p.end()


class Sparkline(QWidget):
    """迷你火花线：滚动显示最近 window_s 秒的数值曲线（自绘 QPainter）。"""

    def __init__(self, window_s: float = 10.0, color: str = ACCENT,
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._window = float(window_s)
        self._color = QColor(color)
        self._points: Deque[Tuple[float, float]] = deque()
        self.setMinimumHeight(30)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def sizeHint(self) -> QSize:  # noqa: N802
        return QSize(120, 32)

    def append(self, value: float, t: Optional[float] = None) -> None:
        """追加一个采样点；NaN 直接忽略。自动裁剪窗口外的旧点。"""
        if value is None or math.isnan(value):
            return
        now = time.monotonic() if t is None else t
        self._points.append((now, float(value)))
        cutoff = now - self._window
        while self._points and self._points[0][0] < cutoff:
            self._points.popleft()
        self.update()

    def clear(self) -> None:
        self._points.clear()
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        # 背景与边框
        p.fillRect(0, 0, w, h, QColor(BG_INPUT))
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawRect(0, 0, w - 1, h - 1)
        pts = list(self._points)
        if len(pts) < 2:
            p.end()
            return
        t1 = pts[-1][0]
        t0 = t1 - self._window
        vmin = min(v for _, v in pts)
        vmax = max(v for _, v in pts)
        span = max(vmax - vmin, 1.0)          # 至少 1° 量程，避免直线抖动
        pad = span * 0.12
        vmin -= pad
        span += pad * 2.0
        margin = 3
        poly = []
        for t, v in pts:
            x = margin + (t - t0) / self._window * (w - margin * 2)
            y = (h - margin) - (v - vmin) / span * (h - margin * 2)
            poly.append((x, y))
        pen = QPen(self._color, 1.4)
        p.setPen(pen)
        p.drawPolyline([QPointF(x, y) for x, y in poly])
        p.end()


class MjpegView(QWidget):
    """MJPEG 流查看器：QNetworkAccessManager 解析 multipart JPEG。

    start(url) 开始拉流，stop() 中止。URL 为空时显示占位提示。
    帧解析策略：在接收缓冲中查找 JPEG SOI(FFD8)/EOI(FFD9) 对，
    每凑齐一帧立即解码显示（精简实现，兼容绝大多数 MJPEG 服务）。
    """

    _BUF_LIMIT = 4 * 1024 * 1024   # 缓冲上限，防止异常流撑爆内存

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._label = QLabel(self)
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setWordWrap(True)
        self._label.setMinimumSize(320, 240)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(self._label)
        self._nam: Optional[QNetworkAccessManager] = None
        self._reply: Optional[QNetworkReply] = None
        self._buf = b''
        self._show_text(tr('Camera stream URL not configured\n(config.json → ssh.camera_url)'))

    # ---- 对外接口 ----
    def start(self, url: str) -> None:
        self.stop()
        url = (url or '').strip()
        if not url:
            self._show_text(tr('Camera stream URL not configured\n(config.json → ssh.camera_url)'))
            return
        self._show_text(tr('Connecting to camera stream…\n%s') % url)
        if self._nam is None:
            self._nam = QNetworkAccessManager(self)
        req = QNetworkRequest(QUrl(url))
        self._reply = self._nam.get(req)
        self._reply.readyRead.connect(self._on_ready_read)
        self._reply.finished.connect(self._on_finished)

    def stop(self) -> None:
        if self._reply is not None:
            reply, self._reply = self._reply, None
            try:
                reply.readyRead.disconnect(self._on_ready_read)
                reply.finished.disconnect(self._on_finished)
            except (RuntimeError, TypeError):
                pass
            reply.abort()
            reply.deleteLater()
        self._buf = b''

    # ---- 内部 ----
    def _show_text(self, text: str) -> None:
        self._label.setPixmap(QPixmap())
        self._label.setText(text)

    def _on_ready_read(self) -> None:
        if self._reply is None:
            return
        self._buf += bytes(self._reply.readAll())
        frame = None
        while True:
            soi = self._buf.find(b'\xff\xd8')
            if soi < 0:
                # 无帧头：只保留尾部 1 字节（防止 FFD8 被截断）
                self._buf = self._buf[-1:]
                break
            eoi = self._buf.find(b'\xff\xd9', soi + 2)
            if eoi < 0:
                if len(self._buf) - soi > self._BUF_LIMIT:
                    self._buf = b''       # 异常流，丢弃重来
                else:
                    self._buf = self._buf[soi:]
                break
            frame = self._buf[soi:eoi + 2]
            self._buf = self._buf[eoi + 2:]
        if frame:
            img = QImage.fromData(frame, 'JPEG')
            if not img.isNull():
                pix = QPixmap.fromImage(img).scaled(
                    self._label.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation)
                self._label.setText('')
                self._label.setPixmap(pix)

    def _on_finished(self) -> None:
        reply = self._reply
        if reply is None:
            return       # stop() 主动中止
        self._reply = None
        if reply.error() != QNetworkReply.NetworkError.NoError:
            self._show_text(tr('Camera stream failed/interrupted:\n%s') % reply.errorString())
        else:
            self._show_text(tr('Camera stream ended'))
        reply.deleteLater()


def confirm(parent: Optional[QWidget], title: str, text: str,
            estop_cb: Optional[Callable[[], None]] = None) -> bool:
    """模态确认对话框（中文按钮）。返回 True 表示用户点了「确认执行」。

    estop_cb：非 None 时对话框内附加一个红色「急停」按钮——模态对话框会挡住
    主窗口工具栏的急停按钮，机械臂仍在运动时用户必须有鼠标可点的急停出口。
    点击后立即调用 estop_cb 并按「取消」处理（返回 False）。
    """
    box = QMessageBox(parent)
    box.setWindowTitle(title)
    box.setIcon(QMessageBox.Icon.Warning)
    box.setText(text)
    ok = box.addButton(tr('Confirm & Execute'), QMessageBox.ButtonRole.AcceptRole)
    cancel = box.addButton(tr('Cancel'), QMessageBox.ButtonRole.RejectRole)
    btn_estop = None
    if estop_cb is not None:
        btn_estop = box.addButton(tr('E-STOP'), QMessageBox.ButtonRole.DestructiveRole)
        btn_estop.setObjectName('estopButton')     # 套用全局 QSS 的大红样式
    box.setDefaultButton(cancel)        # 默认落在「取消」，防误触
    box.exec()
    clicked = box.clickedButton()
    if btn_estop is not None and clicked is btn_estop:
        estop_cb()                      # 急停最优先：立即发出，且不确认原操作
        return False
    return clicked is ok


def repolish(widget: QWidget) -> None:
    """动态属性变化后刷新样式（含所有子控件，用于属性选择器生效）。"""
    widgets = [widget] + widget.findChildren(QWidget)
    for w in widgets:
        w.style().unpolish(w)
        w.style().polish(w)


register({
    'Camera stream URL not configured\n(config.json → ssh.camera_url)':
        '未配置相机流地址\n（config.json → ssh.camera_url）',
    'Connecting to camera stream…\n%s': '正在连接相机流…\n%s',
    'Camera stream failed/interrupted:\n%s': '相机流连接失败/中断：\n%s',
    'Camera stream ended': '相机流已结束',
    'Confirm & Execute': '确认执行',
    'Cancel': '取消',
    'E-STOP': '急  停',
})
