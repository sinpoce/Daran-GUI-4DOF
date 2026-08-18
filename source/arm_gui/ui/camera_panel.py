# -*- coding: utf-8 -*-
"""相机面板：Intel RealSense D435i（RGB / 深度伪彩 / 红外）+ MJPEG 网络流备用。

设计
----
* RealSense 采集在独立 QThread（_RsWorker）：pipeline 同时开 color+depth+infrared
  三路流，切换源只是选择渲染哪一路 —— 切换瞬时完成，无需重启 pipeline。
* pyrealsense2 为可选依赖：启动采集时才 import，缺失时状态栏提示安装命令。
* MJPEG 页复用 widgets.MjpegView（供以后 Jetson 推流用），与 RealSense 页
  放在 QStackedWidget 中按源切换。
* 图像按面板尺寸等比缩放显示；面板隐藏/窗口关闭时自动停止采集释放相机。
"""
from __future__ import annotations

import time
from threading import Event
from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QStackedWidget,
    QVBoxLayout, QWidget,
)

from ..core.config import AppConfig
from ..i18n import register, tr
from .widgets import MjpegView

# 源定义：(combo 文本, worker 源键)
_RS_SOURCES = [('D435i RGB', 'color'),
               ('D435i Depth', 'depth'),
               ('D435i Infrared', 'ir')]
_MJPEG_LABEL = 'MJPEG Stream'

_W, _H, _FPS = 640, 480, 30


class _RsWorker(QThread):
    """RealSense 采集线程：三路流同开，按 self.source 选路渲染。"""

    frame_ready = Signal(QImage)
    status_changed = Signal(str)     # 状态文字（帧率/错误）
    stopped = Signal()

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.source = 'color'        # 'color' | 'depth' | 'ir'（GUI 线程写，本线程读）
        self._stop = Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:   # noqa: C901
        try:
            import numpy as np
            import pyrealsense2 as rs
        except ImportError as e:
            self.status_changed.emit(
                tr('Missing dependency %s — run: python -m pip install pyrealsense2')
                % e.name)
            self.stopped.emit()
            return

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, _W, _H, rs.format.rgb8, _FPS)
        cfg.enable_stream(rs.stream.depth, _W, _H, rs.format.z16, _FPS)
        cfg.enable_stream(rs.stream.infrared, 1, _W, _H, rs.format.y8, _FPS)
        try:
            profile = pipe.start(cfg)
        except Exception as e:
            self.status_changed.emit(
                tr('Failed to open camera: %s (not connected / already in use?)') % e)
            self.stopped.emit()
            return

        name = tr('Unknown device')
        try:
            name = profile.get_device().get_info(rs.camera_info.name)
        except Exception:
            pass
        self.status_changed.emit(tr('%s started') % name)
        colorizer = rs.colorizer()

        n_frames, t_fps = 0, time.monotonic()
        consec_errors = 0
        try:
            while not self._stop.is_set():
                try:
                    frames = pipe.wait_for_frames(timeout_ms=2000)
                    consec_errors = 0
                except Exception as e:
                    consec_errors += 1
                    self.status_changed.emit(tr('Frame timeout: %s') % e)
                    if consec_errors >= 3:
                        # 管线已失效（设备拔出/被占用），不再无限重试
                        self.status_changed.emit(
                            tr('Camera stream lost — press Start to retry'))
                        break
                    continue
                src = self.source
                img: Optional[QImage] = None
                if src == 'color':
                    f = frames.get_color_frame()
                    if f:
                        a = np.asanyarray(f.get_data())
                        img = QImage(a.tobytes(), _W, _H, _W * 3,
                                     QImage.Format.Format_RGB888)
                elif src == 'depth':
                    f = frames.get_depth_frame()
                    if f:
                        a = np.asanyarray(colorizer.colorize(f).get_data())
                        img = QImage(a.tobytes(), _W, _H, _W * 3,
                                     QImage.Format.Format_RGB888)
                else:   # 'ir'
                    f = frames.get_infrared_frame(1)
                    if f:
                        a = np.asanyarray(f.get_data())
                        img = QImage(a.tobytes(), _W, _H, _W,
                                     QImage.Format.Format_Grayscale8)
                if img is not None:
                    # tobytes 缓冲在本作用域即失效，必须 copy 让 QImage 持有数据
                    self.frame_ready.emit(img.copy())
                    n_frames += 1
                now = time.monotonic()
                if now - t_fps >= 1.0:
                    self.status_changed.emit('%s | %d fps' % (name, n_frames))
                    n_frames, t_fps = 0, now
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
            self.stopped.emit()


class CameraPanel(QWidget):
    """相机 Dock 内容：源选择 + 开始/停止 + 图像显示。"""

    def __init__(self, cfg: AppConfig, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._cfg = cfg
        self._worker: Optional[_RsWorker] = None
        self._last_pixmap: Optional[QPixmap] = None

        # ---- 顶部：源选择 + 开始/停止 + 状态 ----
        self.combo = QComboBox()
        for label, _key in _RS_SOURCES:
            self.combo.addItem(tr(label))
        self.combo.addItem(tr(_MJPEG_LABEL))
        self.combo.currentIndexChanged.connect(self._on_source_changed)

        self.btn = QPushButton(tr('Start'))
        self.btn.setMinimumWidth(64)
        self.btn.clicked.connect(self._on_toggle)

        self.status_lbl = QLabel(tr('Not started'))
        self.status_lbl.setObjectName('dimLabel')

        bar = QHBoxLayout()
        bar.setSpacing(6)
        bar.addWidget(QLabel(tr('Source')))
        bar.addWidget(self.combo, 0)
        bar.addWidget(self.btn, 0)
        bar.addWidget(self.status_lbl, 1)

        # ---- 图像区：RealSense 页 / MJPEG 页 ----
        self.image_lbl = QLabel(tr('Not started — select a source and press Start'))
        self.image_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_lbl.setObjectName('imagePlaceholder')
        self.image_lbl.setSizePolicy(QSizePolicy.Policy.Expanding,
                                     QSizePolicy.Policy.Expanding)
        self.image_lbl.setMinimumSize(320, 240)

        self.mjpeg = MjpegView(self)

        self.stack = QStackedWidget()
        self.stack.addWidget(self.image_lbl)    # 页 0：RealSense
        self.stack.addWidget(self.mjpeg)        # 页 1：MJPEG
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)
        lay.addLayout(bar, 0)
        lay.addWidget(self.stack, 1)

    # ================= 源切换 / 开始停止 =================
    def _is_mjpeg(self) -> bool:
        return self.combo.currentIndex() >= len(_RS_SOURCES)

    def _rs_key(self) -> str:
        return _RS_SOURCES[self.combo.currentIndex()][1]

    def _on_source_changed(self) -> None:
        if self._is_mjpeg():
            # 切到 MJPEG 页：停掉 RealSense（若在跑），流地址来自配置
            self.stack.setCurrentIndex(1)
            self._stop_rs()
            if self.btn.text() == tr('Stop'):
                self.mjpeg.start(self._cfg.ssh.camera_url)
        else:
            self.stack.setCurrentIndex(0)
            self.mjpeg.stop()
            if self._worker is not None:
                self._worker.source = self._rs_key()   # 运行中瞬时切路
            elif self.btn.text() == tr('Stop'):
                self._start_rs()

    def _on_toggle(self) -> None:
        if self.btn.text() == tr('Start'):
            self.btn.setText(tr('Stop'))
            if self._is_mjpeg():
                self.mjpeg.start(self._cfg.ssh.camera_url)
                self.status_lbl.setText('MJPEG: %s'
                                        % (self._cfg.ssh.camera_url
                                           or tr('URL not configured')))
            else:
                self._start_rs()
        else:
            self.stop()

    def _start_rs(self) -> None:
        if self._worker is not None:
            self._worker.source = self._rs_key()
            return
        self.status_lbl.setText(tr('Opening camera…'))
        w = _RsWorker(self)
        w.source = self._rs_key()
        w.frame_ready.connect(self._on_frame)
        w.status_changed.connect(self.status_lbl.setText)
        w.stopped.connect(self._on_worker_stopped)
        self._worker = w
        w.start()

    def _stop_rs(self) -> None:
        if self._worker is not None:
            self._worker.stop()

    def stop(self) -> None:
        """停止全部采集（dock 隐藏/窗口关闭/手动停止时调用）。

        同步等待采集线程退出（pipeline.stop 约 0.1~0.3s）：若不等待，
        应用退出时 QThread 还在运行会被强制销毁导致进程崩溃。
        """
        self.btn.setText(tr('Start'))
        self.mjpeg.stop()
        w = self._worker
        if w is not None:
            w.stop()
            w.wait(3000)
        if not self._is_mjpeg():
            self.status_lbl.setText(tr('Stopped'))

    def _on_worker_stopped(self) -> None:
        w = self._worker
        self._worker = None
        if w is not None:
            w.wait(1000)
            w.deleteLater()

    # ================= 图像显示 =================
    def _on_frame(self, img: QImage) -> None:
        self._last_pixmap = QPixmap.fromImage(img)
        self._rescale()

    def _rescale(self) -> None:
        if self._last_pixmap is None:
            return
        self.image_lbl.setPixmap(self._last_pixmap.scaled(
            self.image_lbl.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event) -> None:   # noqa: N802（Qt 命名）
        super().resizeEvent(event)
        self._rescale()


register({
    'D435i Depth': 'D435i 深度',
    'D435i Infrared': 'D435i 红外',
    'MJPEG Stream': 'MJPEG 网络流',
    'Start': '开始',
    'Stop': '停止',
    'Source': '信号源',
    'Not started': '未启动',
    'Not started — select a source and press Start':
        '未启动 —— 选择信号源后点「开始」',
    'Opening camera…': '正在打开相机…',
    'Stopped': '已停止',
    'URL not configured': '未配置地址',
    'Missing dependency %s — run: python -m pip install pyrealsense2':
        '缺少依赖 %s —— 请运行: python -m pip install pyrealsense2',
    'Failed to open camera: %s (not connected / already in use?)':
        '相机打开失败: %s（是否未接入/被占用？）',
    'Unknown device': '未知设备',
    '%s started': '%s 已开启',
    'Frame timeout: %s': '取帧超时: %s',
    'Camera stream lost — press Start to retry': '相机流已中断——点「开始」重试',
})
