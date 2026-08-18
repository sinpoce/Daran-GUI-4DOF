# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller 打包配置 —— DrEmpower 机械臂 GUI

构建：
    pyinstaller ArmGUI.spec --noconfirm

产出：
    dist/ArmGUI.exe         <- 单文件可执行程序

设计要点（都是踩过坑才这么写的）：

1. 使用 onefile，发布时只需要一个 ArmGUI.exe。
   启动时 PyInstaller 会把依赖解压到临时目录，关闭程序后自动清理。

2. config.json 不打进包里，程序首次运行使用默认配置；用户配置和标定结果会写在 exe 同目录。

3. 排除 torch / ultralytics / matplotlib 等。
   GUI 本身完全不 import 它们（YOLO 只在 tools/ 脚本里用）。
   带上会让包从 ~250MB 涨到 1.5GB 以上。

4. 不要让任何东西把 opencv-python（普通版）带进来。
   本项目标定依赖 cv2.aruco，只有 opencv-contrib 才有。
"""
import os
import sys
from PyInstaller.utils.hooks import collect_dynamic_libs, collect_submodules
from PyInstaller.utils.win32.versioninfo import (
    FixedFileInfo, StringFileInfo, StringStruct, StringTable,
    VarFileInfo, VarStruct, VSVersionInfo,
)

block_cipher = None
ROOT = os.path.abspath(SPECPATH)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from arm_gui.version import APP_NAME, WINDOWS_STRING_FILE_INFO, WINDOWS_VERSION

version_info = VSVersionInfo(
    ffi=FixedFileInfo(
        filevers=WINDOWS_VERSION,
        prodvers=WINDOWS_VERSION,
        mask=0x3F,
        flags=0x0,
        OS=0x40004,
        fileType=0x1,
        subtype=0x0,
        date=(0, 0),
    ),
    kids=[
        StringFileInfo([
            StringTable('040904B0', [
                StringStruct(key, value)
                for key, value in WINDOWS_STRING_FILE_INFO.items()
            ]),
        ]),
        VarFileInfo([VarStruct('Translation', [1033, 1200])]),
    ],
)

# ---- 随包只读资源（打包进 _MEIPASS）----
datas = []

# 全局 QSS 使用的 SpinBox 箭头与 Checkbox 勾选图标。PyInstaller 的
# collect_submodules() 只收 Python 模块，不会自动携带这些 SVG 数据文件。
_ui_asset_dir = os.path.join('arm_gui', 'ui', 'assets')
for _asset_name in ('arrow_up.svg', 'arrow_down.svg', 'check.svg'):
    _asset = os.path.join(_ui_asset_dir, _asset_name)
    if os.path.exists(os.path.join(ROOT, _asset)):
        datas.append((_asset, _ui_asset_dir))

# 三维靶模型：core/target3d.py 运行时要读
_model = os.path.join('calibration', 'target', 'target_model.json')
if os.path.exists(os.path.join(ROOT, _model)):
    datas.append((_model, os.path.join('calibration', 'target')))

# 标定板 PDF：标定对话框里"打印标定板"会用到
for _pdf in (os.path.join('target', 'strips_A4.pdf'),):
    _p = os.path.join('calibration', _pdf)
    if os.path.exists(os.path.join(ROOT, _p)):
        datas.append((_p, os.path.join('calibration', 'target')))

# ---- pyrealsense2 的原生 DLL（PyInstaller 自动探测经常漏）----
binaries = []
try:
    binaries += collect_dynamic_libs('pyrealsense2')
except Exception:
    pass

hiddenimports = [
    # 厂商库通过包路径 import，静态分析能看到，这里保险起见显式列出
    'serial', 'serial.tools', 'serial.tools.list_ports',
    'pyrealsense2',
    'yaml',
    'scipy.optimize', 'scipy.spatial',
]
# arm_gui 内部有按需 import（如 i18n、各 panel），全部收进来避免漏
hiddenimports += collect_submodules('arm_gui')

a = Analysis(
    ['main.py'],
    pathex=[ROOT],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # GUI 不依赖，排除后体积从 ~1.5GB 降到 ~250MB
        'torch', 'torchvision', 'ultralytics', 'polars',
        'matplotlib', 'pandas', 'seaborn',
        'IPython', 'jupyter', 'notebook',
        'tkinter', 'PyQt5', 'PyQt6', 'PySide2',
        'pytest', 'sphinx',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    name=APP_NAME,
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,              # UPX 压缩会让部分杀软误报，且对 Qt DLL 有兼容问题
    console=False,          # 无控制台窗口；排障时改 True 可看到 traceback
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    version=version_info,
    # icon='app.ico',       # 有图标的话取消注释
)
