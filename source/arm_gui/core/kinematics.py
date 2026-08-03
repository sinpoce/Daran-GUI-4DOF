# -*- coding: utf-8 -*-
"""Official GUI kinematics for the four-axis arm.

The standard-DH chain, motor mapping and calibrated nail TCP in this module
are shared by FK display, Cartesian IK and hand-eye calibration.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

from . import kinematics_chain as _chain


A1 = _chain.A1_MM
A2 = _chain.A2_MM
A3 = _chain.A3_MM
O3 = 0.0
VERTICAL_PITCH_DEG = -90.0
Z0_WORLD_MM: float | None = 267.9

CAD_TCP_J4_MM = _chain.CAD_TCP_J4_MM.copy()
CALIBRATED_TCP_J4_MM = np.array(
    [227.1253439, -3.13393955, 98.95586891],
    dtype=float,
)
TCP_J4_MM = CALIBRATED_TCP_J4_MM.copy()
TCP_STATUS = (
    "official nail TCP from current-target RGB-D pose and multi-point touch; "
    "touch RMS 2.41 mm, external six-point RMS 3.83 mm, commanded point-20 "
    "hover/contact holdout completed"
)

# Backward-compatible scalar used only by old reach hints.
L_TOOL_MM = float(math.hypot(TCP_J4_MM[0], TCP_J4_MM[1]))


def motor_to_model(m: Sequence[float]) -> List[float]:
    """Raw J1..J4 motor readings -> standard-DH total angles, degrees."""
    return _chain.motor_to_dh(m)


def model_to_motor(j: Sequence[float]) -> List[float]:
    """Standard-DH total angles -> raw J1..J4 motor targets, degrees."""
    return _chain.dh_to_motor(j)


def base_to_gripper(motor_angles: Sequence[float]) -> np.ndarray:
    """Raw motor readings -> T_base_J4, metres."""
    return _chain.base_to_j4(motor_angles)


def fk(j1: float, j2: float, j3: float, j4: float) -> Tuple[float, float, float, float]:
    """DH model angles -> J4 origin and tool pitch."""
    motor = model_to_motor([j1, j2, j3, j4])
    transform = _chain.base_to_j4(motor)
    xyz = transform[:3, 3] * 1000.0
    return float(xyz[0]), float(xyz[1]), float(xyz[2]), float(j2 + j3 + j4)


def fk_tcp(j1: float, j2: float, j3: float, j4: float) -> Tuple[float, float, float, float]:
    """DH model angles -> calibrated nail TCP position and pitch."""
    motor = model_to_motor([j1, j2, j3, j4])
    xyz = _chain.tcp_base_mm(motor, TCP_J4_MM)
    return float(xyz[0]), float(xyz[1]), float(xyz[2]), float(j2 + j3 + j4)


def fk_from_motor(m: Sequence[float]) -> Tuple[float, float, float, float]:
    """Raw motor readings -> calibrated nail TCP position and pitch."""
    return fk_tcp(*motor_to_model(m))


def world_z(z_model: float) -> float | None:
    return None if Z0_WORLD_MM is None else float(z_model) + Z0_WORLD_MM


MODEL_LIMITS = {1: (-160.0, 160.0), 2: (-40.0, 180.0),
                3: (-160.0, 160.0), 4: (-720.0, 720.0)}


def ik(x: float, y: float, z: float, pitch: float,
       current: Sequence[float] | None = None,
       tol_mm: float = 0.5) -> List[float] | None:
    """Calibrated nail TCP IK.

    The return value follows the GUI API: DH/model angles, not raw motor
    readings.  The GUI converts them back through :func:`model_to_motor`.
    """
    current_motor = model_to_motor(current) if current is not None else None
    motor = _chain.ik_tcp(
        [x, y, z],
        pitch,
        current_motor_deg=current_motor,
        tcp_j4_mm=TCP_J4_MM,
    )
    if motor is None:
        return None
    if np.linalg.norm(_chain.tcp_base_mm(motor, TCP_J4_MM) - [x, y, z]) > tol_mm:
        return None
    return motor_to_model(motor)
