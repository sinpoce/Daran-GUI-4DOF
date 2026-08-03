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
from PyInstaller.utils.hooks import collect_dynamic_libs, collect_submodules

block_cipher = None
ROOT = os.path.abspath(os.getcwd())

# ---- 随包只读资源（打包进 _MEIPASS）----
datas = []

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
    name='ArmGUI',
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
    # icon='app.ico',       # 有图标的话取消注释
)
