# -*- coding: utf-8 -*-
"""PySide6 / Qt 加载故障排查（自包含，不依赖任何第三方库）。

用法（用出问题的那个解释器跑）：
    .venv\\Scripts\\python.exe check_qt_env.py

针对的典型报错：
    ImportError: DLL load failed while importing QtCore:
        找不到指定的程序 / The specified procedure could not be found.

Windows 错误码要分清：
    126 找不到指定的模块  -> DLL 缺失（装漏了 / 被杀毒或 OneDrive 拿走了）
    127 找不到指定的程序  -> DLL 找到了但缺少导出符号 -> 版本对不上
"""
from __future__ import annotations

import os
import sys
import sysconfig

_QT_PKGS = ('PySide6', 'PySide6-Essentials', 'PySide6-Addons', 'shiboken6')


def _line(title: str = '') -> None:
    print('-' * 70)
    if title:
        print(title)
        print('-' * 70)


def _versions() -> dict:
    from importlib import metadata
    found = {}
    for name in _QT_PKGS:
        try:
            found[name] = metadata.version(name)
        except Exception:
            found[name] = None
    return found


def report_versions() -> bool:
    """True 表示四个包版本一致。"""
    _line('1. 包版本（四个必须完全一致）')
    found = _versions()
    for name, ver in found.items():
        print('    %-20s %s' % (name, ver or '未安装'))
    installed = [v for v in found.values() if v]
    if not installed:
        print('\n    [X] 一个都没装。')
        return False
    ok = len(set(installed)) == 1 and all(found.values())
    if ok:
        print('\n    [OK] 版本一致。')
    else:
        print('\n    [X] 版本不一致或缺包 —— 这正是 "找不到指定的程序(127)" 的头号原因：')
        print('        QtCore.pyd 要从 shiboken6.abi3.dll 里取符号，')
        print('        版本一对不上，符号表就对不上，DLL 加载了也用不了。')
    return ok


def report_files() -> None:
    _line('2. 关键 DLL 是否就位')
    site = sysconfig.get_paths().get('purelib', '')
    targets = [
        os.path.join(site, 'PySide6', 'Qt6Core.dll'),
        os.path.join(site, 'PySide6', 'QtCore.pyd'),
        os.path.join(site, 'shiboken6', 'shiboken6.abi3.dll'),
    ]
    for p in targets:
        if os.path.exists(p):
            print('    [OK] %-22s %8.1f KB  %s'
                  % (os.path.basename(p), os.path.getsize(p) / 1024.0, p))
        else:
            print('    [X ] %-22s 缺失  %s' % (os.path.basename(p), p))
    binp = os.path.join(site, 'PySide6', 'bin')
    print('\n    PySide6\\bin 是否存在: %s  <- 6.4+ 的 wheel 本来就没有这个目录，'
          '不存在是正常的' % os.path.isdir(binp))


def report_conflicts() -> None:
    _line('3. 环境里其它 Qt6Core.dll（会抢先被加载）')
    seen = []
    site = sysconfig.get_paths().get('purelib', '')
    mine = os.path.normcase(os.path.join(site, 'PySide6'))
    roots = [d for d in os.environ.get('PATH', '').split(os.pathsep) if d.strip()]
    roots += [os.path.join(sys.base_prefix, 'Library', 'bin'),
              os.path.join(sys.prefix, 'Library', 'bin')]
    for d in roots:
        try:
            p = os.path.join(d, 'Qt6Core.dll')
            if os.path.exists(p) and os.path.normcase(d) != mine:
                seen.append(p)
        except Exception:
            continue
    if seen:
        print('    [!] 发现其它 Qt6Core.dll：')
        for p in seen:
            print('        %s' % p)
        print('\n    Python 3.8+ 的扩展模块不再走 PATH 找 DLL，所以通常不致命；')
        print('    但如果启动脚本用 os.add_dll_directory 把这些目录加进去了，就会抢先。')
    else:
        print('    [OK] PATH / conda Library\\bin 里没有其它 Qt6Core.dll。')


def report_environment() -> None:
    _line('4. 运行环境')
    print('    解释器      : %s' % sys.executable)
    print('    Python      : %s' % sys.version.split()[0])
    print('    Anaconda 基座: %s' % ('是' if 'conda' in sys.version.lower()
                                     or 'anaconda' in sys.version.lower() else '否'))
    print('    base_prefix : %s' % sys.base_prefix)
    print('    prefix      : %s' % sys.prefix)
    site = sysconfig.get_paths().get('purelib', '')
    print('    site-packages: %s' % site)
    low = site.lower()
    if 'onedrive' in low:
        print('\n    [!] 项目在 OneDrive 里。按需下载会把 DLL 变成占位文件，')
        print('        同步中还会锁文件 —— Qt 这种一次加载几十个 DLL 的库很容易踩雷。')
        print('        建议：把项目移出 OneDrive，或对该文件夹选「始终保留在此设备上」。')
    if len(site) > 150:
        print('\n    [!] 路径长度 %d，逼近 Windows 260 字符上限，装包时容易被截断。'
              % len(site))
    if ' ' in site:
        print('    [i] 路径里有空格（本身没问题，但某些构建脚本会处理不当）。')


def report_import() -> bool:
    _line('5. 实际导入')
    try:
        import shiboken6
        print('    [OK] import shiboken6  -> %s'
              % os.path.dirname(shiboken6.__file__))
    except Exception as e:
        print('    [X ] import shiboken6 失败: %s: %s' % (type(e).__name__, e))
        return False
    try:
        from PySide6 import QtCore
        print('    [OK] import PySide6.QtCore -> Qt %s / PySide6 %s'
              % (QtCore.qVersion(), QtCore.__version__))
        return True
    except Exception as e:
        print('    [X ] import PySide6.QtCore 失败')
        print('         %s: %s' % (type(e).__name__, e))
        code = getattr(e, 'winerror', None)
        if code is not None:
            print('         WinError %s' % code)
        return False


def main() -> int:
    print('=' * 70)
    print('PySide6 / Qt 加载排查')
    print('=' * 70)
    ver_ok = report_versions()
    report_files()
    report_conflicts()
    report_environment()
    import_ok = report_import()

    _line('结论')
    if import_ok:
        print('    Qt 可以正常加载，问题不在这里。')
        return 0
    print('    Qt 无法加载。按顺序处理：\n')
    print('    1) 干净重装 PySide6 全家桶（版本必须一起来）：')
    print('       "%s" -m pip uninstall -y PySide6 PySide6-Essentials '
          'PySide6-Addons shiboken6 shiboken6-generator' % sys.executable)
    print('       "%s" -m pip cache purge' % sys.executable)
    print('       "%s" -m pip install --no-cache-dir PySide6==6.11.1'
          % sys.executable)
    print('       装完再跑一次本脚本。\n')
    if not ver_ok:
        print('    ^^ 版本已经对不上了，这一步大概率就能解决。\n')
    print('    2) 还不行：把项目整个移出 OneDrive（例如 C:\\dev\\ArmGUI），重建 venv。\n')
    print('    3) 仍不行：装最新的 Microsoft Visual C++ 2015-2022 可再发行组件 (x64)。\n')
    print('    4) 仍不行：用干净的 python.org Python 3.11/3.12 重建 venv，绕开 Anaconda。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
