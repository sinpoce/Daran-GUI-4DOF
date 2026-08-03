# -*- coding: utf-8 -*-
"""关节/机械臂状态数据模型 —— 后端与 UI 之间的唯一数据契约。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List


@dataclass
class JointReading:
    """单个关节的一次读数。角度/转速/力矩为电机原始值（°、r/min、Nm）。"""
    joint_id: int
    name: str = ''
    angle_deg: float = math.nan
    speed_rpm: float = math.nan
    torque_nm: float = math.nan
    online: bool = False

    @property
    def valid(self) -> bool:
        return self.online and not math.isnan(self.angle_deg)


@dataclass
class ArmState:
    """机械臂整体状态快照，由后端周期性发布（state_updated 信号）。"""
    joints: List[JointReading] = field(default_factory=list)
    bus_voltage: float = math.nan     # 母线电压 V（可能仅部分后端提供）
    bus_current: float = math.nan     # q 轴电流 A
    latency_ms: float = math.nan      # 单轮询耗时（通信延迟指标）
    timestamp: float = 0.0            # time.time()
    source: str = ''                  # 'can' | 'ssh-direct' | 'ssh-ros2'
    gripper: 'JointReading | None' = None   # 夹爪电机读数（未配置/离线为 None）

    def joint(self, joint_id: int) -> JointReading | None:
        for j in self.joints:
            if j.joint_id == joint_id:
                return j
        return None

    @property
    def online_count(self) -> int:
        return sum(1 for j in self.joints if j.online)
