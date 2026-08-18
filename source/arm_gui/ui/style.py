# -*- coding: utf-8 -*-
"""浅色工业风主题：颜色常量 + 全局 QSS。

约定：
* 颜色常量供各面板/自绘控件（火花线、LED 等）直接引用；
* QSS 通过 build_qss() 生成，app.setStyleSheet() 一次性应用；
* 等宽数字统一使用 MONO_FAMILY（Consolas），在代码中用 mono_font() 设置。
"""
from __future__ import annotations

import os
from string import Template

# QSS 资源目录（SpinBox 箭头 SVG 等）；QSS url() 用正斜杠绝对路径最稳
_ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'assets').replace('\\', '/')

# ---- 基础色（浅灰底工业风） ----
BG_WINDOW    = '#eef1f5'   # 主窗口底色
BG_PANEL     = '#f7f9fb'   # 面板 / Dock 内容
BG_CARD      = '#ffffff'   # 卡片
BG_CARD_OFF  = '#eff1f4'   # 离线卡片（变灰）
BG_INPUT     = '#ffffff'   # 输入框 / 日志底
BG_TOOLBAR   = '#e3e7ed'   # 工具栏 / 状态栏
BORDER       = '#c9d0da'   # 常规边框
BORDER_LIGHT = '#aeb8c6'   # 深一档边框（卡片、悬停）

# ---- 前景色 ----
FG_TEXT      = '#2a3140'   # 主文字
FG_DIM       = '#66707f'   # 次要文字
FG_FAINT     = '#9aa3b1'   # 失效 / 占位

# ---- 强调与语义色 ----
ACCENT       = '#0e7fa6'   # 强调青（数值、选中）
ACCENT_DIM   = '#0a5f7d'   # 强调青（暗）
BLUE         = '#2563eb'   # 强调蓝
GREEN        = '#1d9e50'   # 正常 / 在线 / 当前模式
ORANGE       = '#d97416'   # 警示橙
YELLOW       = '#a8820f'   # 警告（浅底上加深保证可读）
RED          = '#d83838'   # 错误 / 急停
RED_DARK     = '#a02020'   # 急停按下

# ---- 字体 ----
MONO_FAMILY  = 'Consolas'  # 等宽数字


def mono_font(point_size: int = 10, bold: bool = False):
    """构造等宽数字字体（延迟导入 QtGui，便于纯逻辑测试导入本模块）。"""
    from PySide6.QtGui import QFont
    f = QFont(MONO_FAMILY, point_size)
    f.setBold(bold)
    f.setStyleHint(QFont.StyleHint.Monospace)
    return f


def build_light_palette():
    """Return an explicit light application palette.

    QSS does not cover every native sub-control and viewport on every Qt
    platform.  In particular, Windows dark mode can otherwise leak a dark
    ``Base``/``AlternateBase`` into tables while our stylesheet supplies dark
    text.  A complete application palette gives every widget a light fallback;
    QSS below then adds the industrial styling.
    """
    from PySide6.QtGui import QColor, QPalette

    palette = QPalette()
    colors = {
        QPalette.ColorRole.Window: BG_WINDOW,
        QPalette.ColorRole.WindowText: FG_TEXT,
        QPalette.ColorRole.Base: BG_INPUT,
        QPalette.ColorRole.AlternateBase: '#f2f5f8',
        QPalette.ColorRole.ToolTipBase: '#ffffff',
        QPalette.ColorRole.ToolTipText: FG_TEXT,
        QPalette.ColorRole.Text: FG_TEXT,
        QPalette.ColorRole.Button: '#f2f4f8',
        QPalette.ColorRole.ButtonText: FG_TEXT,
        QPalette.ColorRole.BrightText: '#ffffff',
        QPalette.ColorRole.Highlight: ACCENT,
        QPalette.ColorRole.HighlightedText: '#ffffff',
        QPalette.ColorRole.Link: BLUE,
        QPalette.ColorRole.LinkVisited: ACCENT_DIM,
        QPalette.ColorRole.Light: '#ffffff',
        QPalette.ColorRole.Midlight: '#e5e9ef',
        QPalette.ColorRole.Mid: BORDER,
        QPalette.ColorRole.Dark: BORDER_LIGHT,
        QPalette.ColorRole.Shadow: '#768293',
    }
    if hasattr(QPalette.ColorRole, 'PlaceholderText'):
        colors[QPalette.ColorRole.PlaceholderText] = FG_FAINT
    for role, value in colors.items():
        palette.setColor(QPalette.ColorGroup.All, role, QColor(value))

    disabled = QPalette.ColorGroup.Disabled
    for role in (QPalette.ColorRole.WindowText, QPalette.ColorRole.Text,
                 QPalette.ColorRole.ButtonText):
        palette.setColor(disabled, role, QColor(FG_DIM))
    palette.setColor(disabled, QPalette.ColorRole.Base, QColor('#f0f2f5'))
    palette.setColor(disabled, QPalette.ColorRole.AlternateBase,
                     QColor('#eceff3'))
    palette.setColor(disabled, QPalette.ColorRole.Button, QColor('#eceef2'))
    palette.setColor(disabled, QPalette.ColorRole.Highlight,
                     QColor('#c3d6de'))
    palette.setColor(disabled, QPalette.ColorRole.HighlightedText,
                     QColor('#ffffff'))
    return palette


_QSS = Template("""
/* ================= 全局 ================= */
QMainWindow, QDialog { background: $bg_window; }
QWidget { color: $fg_text; font-size: 9pt; }
QLabel { background: transparent; color: $fg_text; }
QLabel:disabled { color: $fg_dim; }
QToolTip {
    background: #ffffff; color: $fg_text;
    border: 1px solid $border_light; padding: 4px 8px;
}
QLabel#dimLabel { color: $fg_dim; }
QLabel#warningLabel {
    background: #fff8df; color: #765b00; font-weight: 600;
    border: 1px solid #e5c968; border-radius: 4px; padding: 8px 10px;
}
QLabel#statusPanelLabel {
    background: $bg_card; color: $fg_text;
    border: 1px solid $border; border-radius: 4px; padding: 7px 9px;
}
QLabel#statusPanelLabel:disabled {
    background: #f0f2f5; color: $fg_dim;
}
QLabel#imagePlaceholder {
    background: $bg_card; color: $fg_dim;
    border: 1px solid $border; border-radius: 5px;
}

/* 分组框必须显式着色；否则深色系统调色板会从原生样式渗入对话框。 */
QGroupBox {
    background: $bg_panel; color: $fg_text;
    border: 1px solid $border; border-radius: 5px;
    margin-top: 10px; padding: 10px 8px 8px 8px;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 10px; padding: 0 5px;
    background: $bg_panel; color: $fg_text; font-weight: 600;
}
QGroupBox:disabled, QGroupBox:disabled::title { color: $fg_dim; }
QScrollArea#dialogScrollArea { background: $bg_window; border: none; }
QScrollArea#dialogScrollArea QWidget#qt_scrollarea_viewport {
    background: $bg_window;
}
QWidget#dialogContent { background: $bg_window; }

/* ================= 工具栏 ================= */
QToolBar {
    background: $bg_toolbar; border: none;
    border-bottom: 1px solid $border; padding: 4px 6px; spacing: 8px;
}
QToolBar::separator { background: $border; width: 1px; margin: 4px 4px; }

/* 锁定/释放关节：处于该状态的按钮亮绿（modeActive 动态属性） */
QPushButton[modeActive="true"] {
    background: #dff3e7; color: #14572e; font-weight: 600;
    border: 1px solid $green;
}
QPushButton[modeActive="true"]:hover { background: #cfecdb; }
QPushButton[modeActive="true"]:disabled {
    background: #e8f0ea; color: $fg_dim; border-color: $border;
}

/* 急停：永远醒目的红，禁用时保持红色但压暗 */
QPushButton#estopButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                                stop:0 #e04545, stop:1 #c22c2c);
    color: #ffffff; font-size: 13pt; font-weight: 700;
    border: 2px solid #a82626; border-radius: 6px; padding: 6px 30px;
}
QPushButton#estopButton:hover { background: #e35555; }
QPushButton#estopButton:pressed { background: $red_dark; }
QPushButton#estopButton:disabled {
    background: #e6b4b4; color: #712b2b; border: 2px solid #c98585;
}

/* ================= Dock ================= */
QDockWidget { color: $fg_dim; }
QDockWidget::title {
    background: $bg_toolbar; padding: 6px 10px;
    border-bottom: 1px solid $border;
}
QMainWindow::separator { background: $bg_window; width: 4px; height: 4px; }
QMainWindow::separator:hover { background: $accent; }

/* ================= Tab ================= */
QTabWidget::pane { border: 1px solid $border; top: -1px; background: $bg_panel; }
QTabBar::tab {
    background: $bg_toolbar; color: $fg_dim;
    border: 1px solid $border; border-bottom: none;
    padding: 6px 16px; margin-right: 2px;
    border-top-left-radius: 4px; border-top-right-radius: 4px;
}
QTabBar::tab:selected { background: $bg_panel; color: $accent; }
QTabBar::tab:hover:!selected { color: $fg_text; }

/* ================= 按钮 ================= */
QPushButton {
    background: #f2f4f8; color: $fg_text;
    border: 1px solid $border_light; border-radius: 4px; padding: 5px 14px;
}
QPushButton:hover { background: #e6eaf0; }
QPushButton:pressed { background: #d8dee6; }
QPushButton:disabled { background: #eceef2; color: $fg_dim; border-color: $border; }

/* 主操作按钮（全部移动到目标 / 连接） */
QPushButton#primaryButton {
    background: $accent; color: #ffffff; font-weight: 600;
    border: 1px solid $accent_dim;
}
QPushButton#primaryButton:hover { background: #1290bb; }
QPushButton#primaryButton:pressed { background: $accent_dim; }
QPushButton#primaryButton:disabled {
    background: #d5e0e5; color: #56616f; border-color: $border;
}

/* ================= 输入控件 ================= */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background: $bg_input; color: $fg_text;
    border: 1px solid $border; border-radius: 3px; padding: 3px 6px;
    selection-background-color: $accent; selection-color: #ffffff;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 1px solid $accent;
}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {
    color: $fg_dim; background: #f0f2f5;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background: #e6eaf0; border: 1px solid $border; width: 16px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover { background: #d8dee6; }
/* 自定义按钮背景会盖掉 Qt 默认箭头图形，必须显式给箭头图片 */
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
    image: url($asset_dir/arrow_up.svg); width: 8px; height: 5px;
}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
    image: url($asset_dir/arrow_down.svg); width: 8px; height: 5px;
}
QSpinBox::up-arrow:disabled, QDoubleSpinBox::up-arrow:disabled,
QSpinBox::down-arrow:disabled, QDoubleSpinBox::down-arrow:disabled {
    image: none;
}
QComboBox::drop-down { border-left: 1px solid $border; width: 18px; }
QComboBox::down-arrow {
    image: url($asset_dir/arrow_down.svg); width: 8px; height: 5px;
}
QComboBox QAbstractItemView {
    background: #ffffff; color: $fg_text;
    border: 1px solid $border_light;
    selection-background-color: $accent; selection-color: #ffffff;
}

QCheckBox { color: $fg_text; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px; border: 1px solid $border_light;
    border-radius: 2px; background: $bg_input;
}
QCheckBox::indicator:hover { border-color: $accent; }
QCheckBox::indicator:checked {
    image: url($asset_dir/check.svg);
    background: $accent; border-color: $accent;
}
QCheckBox::indicator:unchecked:disabled {
    border-color: $border; background: #e5e9ee;
}
QCheckBox::indicator:checked:disabled {
    image: url($asset_dir/check.svg);
    border-color: #607985; background: #708b96;
}
QCheckBox:disabled { color: $fg_dim; }

/* ================= 表格 ================= */
QTableView, QTableWidget {
    background: $bg_input; color: $fg_text;
    alternate-background-color: #f1f5f8;
    border: 1px solid $border; border-radius: 3px;
    gridline-color: $border;
    selection-background-color: $accent;
    selection-color: #ffffff;
    outline: 0;
}
QTableView::item, QTableWidget::item {
    color: $fg_text; padding: 4px;
}
QTableView::item:selected, QTableWidget::item:selected {
    background: $accent; color: #ffffff;
}
QTableView:disabled, QTableWidget:disabled {
    background: #f0f2f5; color: $fg_dim;
    alternate-background-color: #eceff3;
    selection-background-color: #c3d6de;
    selection-color: #ffffff;
}
QTableView::item:disabled, QTableWidget::item:disabled {
    color: $fg_dim;
}
QHeaderView { background: $bg_toolbar; color: $fg_text; }
QHeaderView::section {
    background: $bg_toolbar; color: $fg_text; font-weight: 600;
    border: none; border-right: 1px solid $border;
    border-bottom: 1px solid $border_light; padding: 5px 7px;
}
QHeaderView::section:disabled {
    background: #e8ebef; color: $fg_dim; border-color: $border;
}
QTableCornerButton::section {
    background: $bg_toolbar; border: none;
    border-right: 1px solid $border;
    border-bottom: 1px solid $border_light;
}

/* ================= 滑块 ================= */
QSlider::groove:horizontal {
    height: 4px; background: #dfe4ea;
    border: 1px solid $border; border-radius: 2px;
}
QSlider::sub-page:horizontal { background: $accent; border-radius: 2px; }
QSlider::handle:horizontal {
    width: 14px; margin: -6px 0; border-radius: 7px;
    background: $accent; border: 1px solid $accent_dim;
}
QSlider::handle:horizontal:hover { background: #1290bb; }
QSlider::handle:horizontal:disabled { background: $fg_faint; border-color: $border; }
QSlider::sub-page:horizontal:disabled { background: $border; }

/* ================= 文本区 / 日志台 ================= */
QPlainTextEdit {
    background: $bg_input; color: $fg_text;
    border: 1px solid $border; border-radius: 3px;
    selection-background-color: $accent; selection-color: #ffffff;
}

/* ================= 关节卡片 ================= */
QFrame#jointCard {
    background: $bg_card;
    border: 1px solid $border; border-radius: 6px;
}
QFrame#jointCard[offline="true"] {
    background: $bg_card_off; border-color: $border;
}
QLabel#angleValue { color: $accent; }
QFrame#jointCard[offline="true"] QLabel#angleValue { color: $fg_faint; }
QLabel#idBadge {
    color: $accent; border: 1px solid $accent; border-radius: 3px;
    padding: 1px 6px; font-size: 8pt;
}
QFrame#jointCard[offline="true"] QLabel#idBadge {
    color: $fg_faint; border-color: $border;
}
QFrame#jointCard[offline="true"] QLabel { color: $fg_faint; }
QFrame#cardLine { background: $border; border: none; max-height: 1px; }

/* ================= 遥测数字块 ================= */
QFrame#telemetryTile {
    background: $bg_card; border: 1px solid $border; border-radius: 5px;
}
QLabel#tileTitle { color: $fg_dim; font-size: 8pt; }
QLabel#tileValue { color: $fg_text; }
QLabel#tileUnit { color: $fg_dim; font-size: 8pt; }

/* ================= 菜单 / 状态栏 ================= */
QMenuBar { background: $bg_toolbar; color: $fg_text; border-bottom: 1px solid $border; }
QMenuBar::item { padding: 4px 10px; background: transparent; }
QMenuBar::item:selected { background: #d3dae3; }
QMenuBar::item:disabled { color: $fg_dim; background: transparent; }
QMenu { background: #ffffff; color: $fg_text; border: 1px solid $border_light; }
QMenu::item { color: $fg_text; padding: 5px 24px; }
QMenu::item:selected { background: $accent; color: #ffffff; }
QMenu::item:disabled { background: transparent; color: $fg_dim; }
QMenu::separator { height: 1px; background: $border; margin: 4px 8px; }

QStatusBar { background: $bg_toolbar; color: $fg_dim; border-top: 1px solid $border; }
QStatusBar QLabel { color: $fg_dim; }
QLabel#statusError { color: $red; }

/* ================= 滚动条 ================= */
QScrollBar:vertical { background: $bg_window; width: 10px; margin: 0; }
QScrollBar::handle:vertical {
    background: #c2cad4; border-radius: 5px; min-height: 24px;
}
QScrollBar::handle:vertical:hover { background: #aab4c1; }
QScrollBar:horizontal { background: $bg_window; height: 10px; margin: 0; }
QScrollBar::handle:horizontal {
    background: #c2cad4; border-radius: 5px; min-width: 24px;
}
QScrollBar::handle:horizontal:hover { background: #aab4c1; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }

/* ================= 消息框 ================= */
QMessageBox { background: $bg_panel; }
""")


def build_qss() -> str:
    """生成全局 QSS 字符串。"""
    return _QSS.substitute(
        bg_window=BG_WINDOW, bg_panel=BG_PANEL, bg_card=BG_CARD,
        bg_card_off=BG_CARD_OFF, bg_input=BG_INPUT, bg_toolbar=BG_TOOLBAR,
        border=BORDER, border_light=BORDER_LIGHT,
        fg_text=FG_TEXT, fg_dim=FG_DIM, fg_faint=FG_FAINT,
        accent=ACCENT, accent_dim=ACCENT_DIM,
        green=GREEN, orange=ORANGE, red=RED, red_dark=RED_DARK,
        asset_dir=_ASSET_DIR,
    )
