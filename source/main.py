# -*- coding: utf-8 -*-
"""DrEmpower 机械臂控制台入口。

用法：
    python main.py                       正常启动
    python main.py --config my.json      指定配置文件
    python main.py --screenshot out.png  启动 3 秒后截图保存并退出
                                         （配合 QT_QPA_PLATFORM=offscreen
                                           可做无人值守界面验证）
"""
from __future__ import annotations

import argparse
import os
import sys

# 确保无论从哪里运行都能 import arm_gui 包
_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def main() -> int:
    parser = argparse.ArgumentParser(description='DrEmpower 机械臂控制台')
    parser.add_argument('--config', metavar='PATH', default=None,
                        help='配置文件路径（默认：项目根目录 config.json）')
    parser.add_argument('--screenshot', metavar='PATH', default=None,
                        help='启动 3 秒后截图保存为 PNG 并退出（无人值守验证）')
    args = parser.parse_args()

    from PySide6.QtCore import QTimer
    from PySide6.QtGui import QFont
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    app.setOrganizationName('DrEmpower')
    app.setApplicationName('ArmGUI')
    if sys.platform.startswith('win'):
        app.setFont(QFont('Microsoft YaHei UI', 9))

    from arm_gui.ui.style import build_qss
    app.setStyleSheet(build_qss())

    from arm_gui.core.config import AppConfig
    cfg = AppConfig.load(args.config) if args.config else AppConfig.load()

    # 语言必须在构建任何窗口之前设定（默认英文，config.json ui.language 可改）
    from arm_gui.i18n import set_language
    set_language(cfg.ui.language)

    from arm_gui.ui.main_window import MainWindow
    win = MainWindow(cfg, config_path=args.config)
    win.show()

    if args.screenshot:
        def _grab() -> None:
            pix = win.grab()
            ok = pix.save(args.screenshot, 'PNG')
            print('screenshot %s: %s' % ('saved' if ok else 'FAILED',
                                         args.screenshot))
            app.quit()
        QTimer.singleShot(3000, _grab)

    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
