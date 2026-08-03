# DrEmpower Arm GUI（CAN 直连版）

面向 DrEmpower 四轴机械臂的桌面控制 GUI。当前发布版只包含本机 USB-CAN 直连，不包含 SSH、论文实验数据、自动分拣或 YOLO 工具。

## 功能

- J1-J4 关节控制、状态监视、锁定/释放和急停。
- Intel RealSense D435i RGB-D 相机显示。
- 三维标定板手眼标定与标定点测试。
- Manual Point Recorder 手动记录机械臂点位。
- 内置四轴正/逆运动学、软限位和运动安全检查。

## 运行环境

- Windows 10/11
- Python 3.11
- Intel RealSense D435i（使用相机和标定功能时）
- USB-CAN 串口连接机械臂

安装依赖：

```powershell
python -m pip install -r requirements.txt
python main.py
```

注意：必须使用 `opencv-contrib-python`，不要同时安装普通版 `opencv-python`，否则 `cv2.aruco` 会缺失。

## 打包 EXE

在本目录运行：

```powershell
build_exe.bat
```

产物位于 `dist/ArmGUI/`，运行时需要保留整个目录，而不是只复制 `ArmGUI.exe`。

## 目录说明

```text
main.py                 GUI 入口
config.json             本机 CAN 和界面配置
arm_gui/core/           CAN 后端、运动学、标定和状态模型
arm_gui/ui/             PySide6 界面
arm_gui/vendor/         DrEmpower 厂商协议库
calibration/target/     三维标定模型和标定板资源
```

## 安全提示

连接后默认不发送运动命令。执行运动前确认机械臂工作空间无障碍，并随时准备使用 GUI 的 E-STOP 或键盘 Esc 急停。释放关节会关闭保持力矩，机械臂可能下垂，操作前必须托住机械臂。
