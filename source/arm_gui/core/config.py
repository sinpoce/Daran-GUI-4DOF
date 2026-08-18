# -*- coding: utf-8 -*-
"""应用配置：JSON 持久化（config.json，位于项目根目录）。"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Tuple

# 打包(PyInstaller)后 __file__ 指向临时解压目录 _MEIPASS，配置写进去会随程序退出
# 一起消失。用 app_dir() 统一解析：源码运行=项目根，打包后=exe 所在目录。
from arm_gui.core.apppaths import user_path

CONFIG_PATH = user_path('config.json')


@dataclass
class JointConfig:
    joint_id: int
    name: str
    min_deg: float = -175.0
    max_deg: float = 175.0


@dataclass
class CanConfig:
    port: str = 'COM4'
    baud: int = 115200
    poll_hz: float = 20.0


@dataclass
class SshConfig:
    host: str = ''
    port: int = 22
    username: str = ''
    password: str = ''            # 仅当 remember_password 为 True 时持久化
    key_path: str = ''            # 私钥路径，优先于密码
    remember_password: bool = False
    remote_serial_port: str = '/dev/ttyACM0'
    mode: str = 'direct'          # 'direct'=代理直连串口 | 'ros2'=经 ROS2 话题
    ros2_setup_cmd: str = 'source /opt/ros/humble/setup.bash'
    remote_python: str = 'python3'
    poll_hz: float = 20.0
    camera_url: str = ''          # MJPEG 流地址（可经 SSH 隧道），留空不启用


@dataclass
class SafetyConfig:
    max_speed_rpm: float = 15.0
    max_accel: float = 20.0
    max_step_deg: float = 30.0
    default_speed_rpm: float = 8.0
    default_accel: float = 10.0


@dataclass
class UiConfig:
    language: str = 'en'          # 'en' | 'zh'（默认英文，菜单可切换）
    # v1.2 正式启用示教录制、文件持久化和轨迹回放；如现场需要临时
    # 隐藏入口，可在本机 config.json 中把本项改成 false。
    enable_teach_playback: bool = True


@dataclass
class GripperConfig:
    motor_id: int = 6             # 夹爪电机 CAN ID（2026-07-18 扫描确认为 6；0=禁用）
    default_speed_rpm: float = 10.0
    default_accel: float = 20.0


@dataclass
class CalibrationPointConfig:
    """Saved calibration pose (raw J1-J4 motor angles in degrees)."""
    motor_angles_deg: List[float] = field(default_factory=list)
    captured_at: str = ''
    source: str = ''
    sample_count: int = 0
    std_deg: List[float] = field(default_factory=list)


@dataclass
class PlacementPointConfig:
    """A taught class-specific placement pose in raw motor degrees."""
    motor_angles_deg: List[float] = field(default_factory=list)
    captured_at: str = ''
    source: str = ''
    source_image: str = ''
    source_sha256: str = ''


@dataclass
class GraspTestConfig:
    """Settings for the guarded YOLO pick-and-place test."""
    detection_pose_source: str = 'calibration_point'
    release_min_angle_deg: float = 15.0
    release_clearance_deg: float = 1.0
    release_max_angle_deg: float = 20.0
    placement_approach_clearance_mm: float = 70.0
    placement_points: Dict[str, PlacementPointConfig] = field(
        default_factory=dict)


@dataclass
class AppConfig:
    joints: List[JointConfig] = field(default_factory=lambda: [
        JointConfig(1, 'joint1'), JointConfig(2, 'joint2'),
        JointConfig(3, 'joint3'), JointConfig(4, 'joint4'),
    ])
    can: CanConfig = field(default_factory=CanConfig)
    ssh: SshConfig = field(default_factory=SshConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    ui: UiConfig = field(default_factory=UiConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    calibration_point: CalibrationPointConfig = field(
        default_factory=CalibrationPointConfig)
    grasp_test: GraspTestConfig = field(default_factory=GraspTestConfig)
    last_backend: str = 'can'     # 'can' | 'ssh'

    # ---- 便捷访问 ----
    @property
    def joint_ids(self) -> List[int]:
        return [j.joint_id for j in self.joints]

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def angle_limits(self) -> Dict[int, Tuple[float, float]]:
        return {j.joint_id: (j.min_deg, j.max_deg) for j in self.joints}

    # ---- 持久化 ----
    def save(self, path: str = CONFIG_PATH) -> None:
        data = asdict(self)
        if not self.ssh.remember_password:
            data['ssh']['password'] = ''
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str = CONFIG_PATH) -> 'AppConfig':
        cfg = cls()
        if not os.path.exists(path):
            return cfg
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            cfg.joints = [JointConfig(**j) for j in data.get('joints', [])] or cfg.joints
            cfg.can = CanConfig(**{**asdict(cfg.can), **data.get('can', {})})
            cfg.ssh = SshConfig(**{**asdict(cfg.ssh), **data.get('ssh', {})})
            cfg.safety = SafetyConfig(**{**asdict(cfg.safety), **data.get('safety', {})})
            ui_data = {**asdict(cfg.ui), **data.get('ui', {})}
            # ``enable_teach_playback=false`` was written by development
            # builds while the feature was intentionally hidden.  v1.2 ships
            # the completed workflow, so migrate that stale local switch to
            # the release default instead of letting an old config hide it.
            ui_data['enable_teach_playback'] = True
            cfg.ui = UiConfig(**ui_data)
            cfg.gripper = GripperConfig(**{**asdict(cfg.gripper),
                                           **data.get('gripper', {})})
            cfg.calibration_point = CalibrationPointConfig(
                **{**asdict(cfg.calibration_point),
                   **data.get('calibration_point', {})})
            grasp_data = data.get('grasp_test', {})
            placement_points = {
                str(name): PlacementPointConfig(**value)
                for name, value in grasp_data.get(
                    'placement_points', {}).items()
            }
            grasp_scalars = {
                key: value for key, value in grasp_data.items()
                if key != 'placement_points'
            }
            cfg.grasp_test = GraspTestConfig(
                **{**asdict(cfg.grasp_test), **grasp_scalars,
                   'placement_points': placement_points})
            cfg.last_backend = data.get('last_backend', cfg.last_backend)
        except Exception as e:
            print('配置文件加载失败，使用默认配置:', e)
        return cfg
