# Daran-GUI-4DOF v1.2


`关节 Python 函数库-251109` 参考目录或 SSH 远程控制代码。运行时只使用
`source/arm_gui/vendor/drempower/` 中的内置协议模块。

## 1. 项目结构

```text
Daran-GUI-4DOF/
├─ source/                              # Python 源码与构建文件
│  ├─ main.py                           # 程序入口
│  ├─ config.json                       # 首次运行生成的本机配置（不提交）
│  ├─ requirements.txt                  # GUI 运行依赖
│  ├─ run_gui.bat                       # 源码模式启动脚本
│  ├─ build_exe.bat                     # PyInstaller 单文件打包脚本
│  ├─ ArmGUI.spec                       # PyInstaller onefile 配置
│  ├─ README.md                         # 本文档
│  │
│  ├─ arm_gui/                          # GUI Python 包
│  │  ├─ __init__.py                    # 包初始化
│  │  ├─ i18n.py                        # 中英文界面文本注册与切换
│  │  │
│  │  ├─ core/                          # 与界面解耦的控制、模型和标定核心
│  │  │  ├─ apppaths.py                 # 源码/EXE 的资源路径与用户数据路径
│  │  │  ├─ backend_base.py             # 机械臂后端抽象接口和安全校验
│  │  │  ├─ can_backend.py              # USB-CAN 串口后端、轮询、运动命令和急停
│  │  │  ├─ config.py                   # JSON 配置读取、默认值和持久化
│  │  │  ├─ state.py                    # J1–J4 遥测状态数据结构
│  │  │  ├─ kinematics.py               # 四轴 FK/IK、角度映射和 TCP 位姿
│  │  │  ├─ kinematics_chain.py         # 运动学链和雅可比相关计算
│  │  │  ├─ calibration.py              # RGB-D/IMU 标定采集与数值质量检查
│  │  │  ├─ target3d.py                 # 三维标定板模型和角点定义
│  │  │  ├─ anchor_fusion.py             # 多平面锚点融合和位姿优化
│  │  │  ├─ tcp_calibration.py           # 刚性探针/TCP 独立标定辅助
│  │  │  └─ paper_archive.py             # 标定证据归档；不包含论文正文或实验结果
│  │  │
│  │  ├─ ui/                            # PySide6 界面层
│  │  │  ├─ main_window.py              # 主窗口、菜单、连接生命周期和全局急停
│  │  │  ├─ connection_panel.py         # CAN 端口、波特率和连接控制
│  │  │  ├─ joint_panel.py              # J1–J4 目标角度、点动、锁定/释放
│  │  │  ├─ telemetry_panel.py          # 电压、电流、在线关节和 TCP 位姿显示
│  │  │  ├─ camera_panel.py             # D435i RGB/深度/IR 画面
│  │  │  ├─ calibration_dialog.py       # 三维手眼标定和标定点测试页面
│  │  │  ├─ point_recorder_dialog.py    # Manual Point Recorder 手动点位记录
│  │  │  ├─ widgets.py                  # 通用控件、确认框和界面辅助函数
│  │  │  ├─ style.py                    # 工业风 QSS、颜色和字体
│  │  │  └─ assets/                     # 界面 SVG 图标资源
│  │  │
│  │  └─ vendor/drempower/              # DrEmpower 厂商 CAN/串口协议库
│  │     ├─ DrEmpower.py                # 电机读写、角度命令、状态和急停协议
│  │     └─ parameter_interface.py      # 厂商参数接口
│  │
│  └─ calibration/target/                # 随 GUI 发布的只读标定资源
│     ├─ target_model.json              # 三维标定板 CAD/实测角点模型
│     ├─ flat_target_model.json         # 平面模型备用定义
│     ├─ strips_A4.pdf                  # 可打印标定板
│     ├─ multi_plane_target.stl         # 三维标定板模型
│     └─ *_preview.png / README.md      # 预览图和资源说明
│
└─ GitHub Release                       # ArmGUI-v1.2-windows-x64.exe
```

## 2. 软件架构

```text
main.py
   │
   ▼
MainWindow ─────── UI panels/dialogs
   │                         │
   │                         ├─ CameraPanel ── Intel RealSense D435i
   │                         ├─ CalibrationDialog ── calibration.py/target3d.py
   │                         └─ PointRecorderDialog ── state.py/kinematics.py
   │
   ▼
CanBackend ── DrEmpower vendor protocol ── USB-CAN ── J1/J2/J3/J4
```

控制命令只从 `CanBackend` 发出，界面层不直接操作串口。所有运动目标在后端统一经过连接状态、软限位、速度、加速度和单步幅度检查；急停会清空命令队列并关闭运动使能。

## 3. 环境要求

- Windows 10/11
- Python 3.11/3.12 64-bit（仅源码运行或自行构建时需要）
- USB-CAN 串口适配器
- Intel RealSense D435i（使用相机和手眼标定时）

安装依赖：

```powershell
python -m pip install -r requirements.txt
```

必须安装 `opencv-contrib-python`，不要同时安装普通版 `opencv-python`，否则 `cv2.aruco` 标定模块会缺失。

## 4. 源码运行

```powershell
python main.py
```

或双击：

```text
run_gui.bat
```

首次运行会使用英文界面和安全默认值，并在本机生成 `config.json`。在界面中确认
CAN 串口和波特率后再连接；连接后不会自动发送运动命令。

## 5. 单文件 EXE 打包

在 `source` 目录执行：

```powershell
build_exe.bat
```

产物为：

```text
dist/ArmGUI.exe
```

`ArmGUI.spec` 使用 PyInstaller `onefile` 模式，Qt、OpenCV、SciPy、RealSense DLL 和只读标定资源都会封装进 EXE。首次启动时会临时解压依赖，因此启动速度比源码模式慢一些。

目标电脑不需要安装 Python、Anaconda、PySide6 或 requirements 中的包；USB-CAN
与 RealSense 仍需要对应的 Windows 硬件驱动。正式 EXE 作为 GitHub Release
`v1.2` 的附件发布，不放入 Git 源码历史。

运行过程中，用户配置和新生成的标定结果可能写入 EXE 所在目录；这些是运行数据，不属于源码。

## 6. 三维手眼标定资源

`calibration/target/target_model.json` 是 GUI 读取的三维靶标模型，`strips_A4.pdf` 用于打印标定板。标定结果和采集记录不随 GitHub 源码发布，运行后由 GUI 在用户目录生成。

流程：

1. 固定标定板和相机。
2. 连接 CAN 并确认 J1–J4 在线。
3. 在 Calibration 页面设置标定点。
4. 按提示采集多姿态 RGB-D/IMU 数据。
5. 检查重投影、深度、重力和雅可比条件数。
6. 保存结果后使用 Test Points 做独立验证。

## 7. 安全注意事项

- 机械臂首次连接后先确认关节角度和软限位。
- 执行运动前清空工作空间，并保持手能够触发 E-STOP。
- 锁定关节用于闭环保持；释放关节会关闭保持力矩，机械臂可能下垂，必须先托住机械臂。
- 不要在未确认标定板、TCP 和夹爪姿态时执行抓取动作。
- 键盘 `Esc` 和 GUI 的 E-STOP 始终优先于普通运动命令。
