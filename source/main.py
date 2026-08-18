# -*- coding: utf-8 -*-
"""DrEmpower 机械臂控制台入口。

用法：
    python main.py                       正常启动
    python main.py --config my.json      指定配置文件
    python main.py --screenshot out.png  启动 3 秒后截图保存并退出
                                         （配合 QT_QPA_PLATFORM=offscreen
                                           可做无人值守界面验证）
    python main.py --skip-deps           跳过启动自检与自动安装

依赖自检分两级
--------------
* 必需依赖（_REQUIRED_PACKAGES）：界面/通信的地基，缺了直接弹窗报错并退出。
* 可选依赖（_OPTIONAL_PACKAGES）：相机、标定、离线分析用；缺了只打印降级提示，
  机械臂连接与关节控制照常可用——不能因为装不上相机库就让整台机器没法开。

自检只用 importlib.util.find_spec 判断"装没装"，不真正 import：
真 import 会把"装了但导入失败"（缺 VC++ 运行库、没插相机、wheel 与
Python 版本不匹配）误判成"没装"，于是每次启动都重装一遍、再抛异常退出，
GUI 永远起不来。安装日志写到 install_deps.log，报错弹窗会指向它。
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
from typing import Dict, List, Tuple

# 确保无论从哪里运行都能 import arm_gui 包
_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

_REQ_PATH = os.path.join(_ROOT, 'requirements.txt')
_INSTALL_LOG = os.path.join(_ROOT, 'install_deps.log')

from arm_gui.version import APP_NAME, APP_VERSION

# {发行包名: 顶层模块名}
_REQUIRED_PACKAGES: Dict[str, str] = {
    'PySide6': 'PySide6',       # 界面
    'numpy': 'numpy',           # 运动学/遥测面板
    'pyserial': 'serial',       # USB-CAN 串口
}
_OPTIONAL_PACKAGES: Dict[str, str] = {
    'opencv-contrib-python': 'cv2',      # 相机画面与标定（需 contrib 版的 aruco）
    'pyrealsense2': 'pyrealsense2',      # RealSense D435i
    'scipy': 'scipy',                    # 标定最小二乘
    'PyYAML': 'yaml',                    # 标定结果导出
}


def _is_installed(module: str) -> bool:
    """模块是否可被找到（只查元数据，不执行模块代码）。"""
    try:
        return importlib.util.find_spec(module) is not None
    except Exception:
        # 包目录残缺/命名空间包异常等：按已安装处理，交由真正 import 时报错，
        # 避免在这里误触发重装
        return True


def _missing(packages: Dict[str, str]) -> List[str]:
    return [pkg for pkg, mod in packages.items() if not _is_installed(mod)]


def _pinned_specs(packages: List[str]) -> List[str]:
    """从 requirements.txt 取这些包的版本钉死写法，取不到就用裸包名。"""
    pinned: Dict[str, str] = {}
    try:
        with open(_REQ_PATH, 'r', encoding='utf-8') as f:
            for raw in f:
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                name = line
                for sep in ('==', '>=', '<=', '~=', '>', '<', '!=', '['):
                    idx = line.find(sep)
                    if idx > 0:
                        name = line[:idx]
                        break
                pinned[name.strip().lower()] = line
    except OSError:
        pass
    return [pinned.get(pkg.lower(), pkg) for pkg in packages]


def _log_install(header: str, body: str) -> None:
    try:
        with open(_INSTALL_LOG, 'a', encoding='utf-8') as f:
            f.write('\n===== %s =====\n%s\n' % (header, body))
    except OSError:
        pass


def _pip_install(packages: List[str]) -> Tuple[bool, str]:
    """安装给定包，返回 (是否成功, 输出)。全部异常都收敛成返回值。"""
    cmd = [sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check',
           '--no-input'] + _pinned_specs(packages)
    print('[Startup] %s' % ' '.join(cmd[1:]))
    kwargs = {}
    if sys.platform.startswith('win'):
        # pythonw 启动时不要弹出 pip 的黑色控制台窗口
        kwargs['creationflags'] = 0x08000000     # CREATE_NO_WINDOW
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              encoding='utf-8', errors='replace', **kwargs)
    except Exception as e:      # pip 不存在 / 解释器路径异常
        out = '%s: %s' % (type(e).__name__, e)
        _log_install(' '.join(cmd), out)
        return False, out
    out = (proc.stdout or '') + (proc.stderr or '')
    _log_install(' '.join(cmd), out)
    return proc.returncode == 0, out


def _alert(title: str, text: str) -> None:
    """致命错误提示：控制台 + Windows 弹窗。

    run_gui.bat 优先用 pythonw 启动（没有控制台），只 print 的话用户双击后
    "什么都没发生"。弹窗只用 ctypes，不依赖任何第三方库。
    """
    print(text, file=sys.stderr)
    if sys.platform.startswith('win'):
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None, text, title, 0x10)
        except Exception:
            pass


def _manual_hint(packages: List[str]) -> str:
    return '%s -m pip install %s' % (
        os.path.basename(sys.executable) or 'python',
        ' '.join(_pinned_specs(packages)))


def _ensure_runtime_dependencies(auto_install: bool = True) -> None:
    """启动自检：必需依赖缺失则报错退出，可选依赖缺失只降级提示。"""
    if getattr(sys, 'frozen', False):
        return              # 打包版依赖已随包，不做任何自检/安装

    missing_req = _missing(_REQUIRED_PACKAGES)
    missing_opt = _missing(_OPTIONAL_PACKAGES)
    if not missing_req and not missing_opt:
        return

    if auto_install:
        if not os.path.exists(_REQ_PATH):
            print('[Startup] requirements.txt not found: %s' % _REQ_PATH)
        # 必需与可选分两次装：可选包（如无对应 wheel 的 pyrealsense2）安装失败
        # 不能连累必需包，否则一个相机库就把整个程序挡在门外
        for group, pkgs in (('required', missing_req), ('optional', missing_opt)):
            if not pkgs:
                continue
            print('[Startup] Missing %s dependencies: %s'
                  % (group, ', '.join(pkgs)))
            ok, _out = _pip_install(pkgs)
            if not ok:
                print('[Startup] Install of %s dependencies failed, see %s'
                      % (group, _INSTALL_LOG))
        importlib.invalidate_caches()
        missing_req = _missing(_REQUIRED_PACKAGES)
        missing_opt = _missing(_OPTIONAL_PACKAGES)

    if missing_req:
        _alert('ArmGUI', (
            '缺少必需依赖，无法启动：\n    %s\n\n'
            '请手动安装后重试：\n    %s\n\n'
            '安装日志：%s'
        ) % (', '.join(missing_req), _manual_hint(missing_req), _INSTALL_LOG))
        raise SystemExit(1)

    if missing_opt:
        print('[Startup] Optional dependencies unavailable: %s'
              % ', '.join(missing_opt))
        print('[Startup] Camera / calibration features will be unavailable; '
              'arm control works normally.')
        print('[Startup] To retry manually: %s' % _manual_hint(missing_opt))


def main() -> int:
    parser = argparse.ArgumentParser(description='DrEmpower 机械臂控制台')
    parser.add_argument('--config', metavar='PATH', default=None,
                        help='配置文件路径（默认：项目根目录 config.json）')
    parser.add_argument('--screenshot', metavar='PATH', default=None,
                        help='启动 3 秒后截图保存为 PNG 并退出（无人值守验证）')
    parser.add_argument('--skip-deps', action='store_true',
                        help='跳过启动时的依赖自检与自动安装')
    args = parser.parse_args()

    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtGui import QFont
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    app.setOrganizationName('DrEmpower')
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    # Use one deterministic light base on every PC.  Without an explicit
    # style/palette, Windows dark mode can supply dark native table viewports
    # underneath our dark-text QSS, making dialog content unreadable.
    try:
        app.styleHints().setColorScheme(Qt.ColorScheme.Light)
    except (AttributeError, TypeError):
        # Older Qt releases do not expose setColorScheme; Fusion + the
        # explicit palette below still keep all client-area widgets light.
        pass
    app.setStyle('Fusion')
    if sys.platform.startswith('win'):
        app.setFont(QFont('Microsoft YaHei UI', 9))

    from arm_gui.ui.style import build_light_palette, build_qss
    app.setPalette(build_light_palette())
    app.setStyleSheet(build_qss())

    from arm_gui.core.config import AppConfig
    cfg = AppConfig.load(args.config) if args.config else AppConfig.load()

    # 语言必须在构建任何窗口之前设定（默认英文，config.json ui.language 可改）
    from arm_gui.i18n import set_language
    set_language(cfg.ui.language)

    from arm_gui.ui.main_window import MainWindow
    win = MainWindow(cfg, config_path=args.config)
    # Use the platform's native maximise operation when the window is shown.
    # On Windows, setting only WindowMaximized during construction can be
    # overridden by the first show when saved geometry and per-monitor DPI are
    # involved.  This exactly matches clicking the native maximise button.
    win.showMaximized()

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
    # 自检要早于 argparse：argparse 出错也不该绕过依赖检查
    _ensure_runtime_dependencies(auto_install='--skip-deps' not in sys.argv)
    sys.exit(main())
