# Daran GUI 4DOF v1.2

面向 DrEmpower 四轴机械臂的桌面控制 GUI。当前发布版只包含本机 USB-CAN 直连，不包含 SSH、论文实验数据、自动分拣或 YOLO 工具。

## 功能

- J1-J4 关节控制、状态监视、锁定/释放和急停。
- Intel RealSense D435i RGB-D 相机显示。
- 三维标定板手眼标定与标定点测试。
- Manual Point Recorder 手动记录机械臂点位。
- 示教轨迹录制、保存、加载和连续回放。
- 独立 Set Zero 工具，可安全地为选定的机械臂关节设置零点。
- 内置四轴正/逆运动学、软限位和运动安全检查。

## 运行环境

- Windows 10/11
- Python 3.11/3.12 64-bit（仅源码运行或自行构建时需要）
- Intel RealSense D435i（使用相机和标定功能时）
- USB-CAN 串口连接机械臂

安装依赖：

```powershell
python -m pip install -r requirements.txt
python main.py
```

注意：必须使用 `opencv-contrib-python`，不要同时安装普通版 `opencv-python`，否则 `cv2.aruco` 会缺失。

### 启动自检与自动安装

`main.py` 启动时会检查依赖，缺什么装什么，分两级处理：

| 级别 | 依赖 | 缺失时 |
| --- | --- | --- |
| 必需 | PySide6、numpy、pyserial | 尝试安装；仍缺则弹窗提示手动安装命令并退出 |
| 可选 | opencv-contrib-python、pyrealsense2、scipy、PyYAML | 尝试安装；装不上只打印提示，相机/标定功能不可用，机械臂控制照常 |

自检只查包是否安装、不真正 import，因此"装了但导入失败"（缺 VC++ 运行库、
相机 SDK 未装、wheel 与 Python 版本不匹配）不会被误判成未安装、每次启动重装一遍。
安装过程写入 `install_deps.log`。用 `python main.py --skip-deps` 可跳过整个自检。

## 打包 EXE

在本目录运行：

```powershell
build_exe.bat
```

构建脚本会优先使用 `.venv`，并按 `requirements-build.txt` 安装固定版本的
PyInstaller。产物是单文件 `dist/ArmGUI.exe`；目标电脑不需要安装 Python、
PySide6 或项目依赖。USB-CAN 和 RealSense 硬件仍需要相应的 Windows 驱动。

正式的 v1.2 可执行程序在 GitHub Releases 中以
`ArmGUI-v1.2-windows-x64.exe` 发布，不提交进 Git 源码历史。

## 目录说明

```text
main.py                 GUI 入口
config.json             首次运行后生成的本机配置（不提交到 Git）
arm_gui/core/           CAN 后端、运动学、标定和状态模型
arm_gui/ui/             PySide6 界面
arm_gui/vendor/         DrEmpower 厂商协议库
calibration/target/     三维标定模型和标定板资源
```

## 安全提示

连接后默认不发送运动命令。执行运动前确认机械臂工作空间无障碍，并随时准备使用 GUI 的 E-STOP 或键盘 Esc 急停。释放关节会关闭保持力矩，机械臂可能下垂，操作前必须托住机械臂。

## 发布边界

运行时关节协议由 `arm_gui/vendor/drempower/` 提供。程序不会导入、执行或读取
开发时使用的 `关节 Python 函数库-251109/` 参考目录；该目录、`tests/`、虚拟环境、
构建缓存和用户录制数据均不随源码或 EXE 发布。
