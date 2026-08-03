# -*- coding: utf-8 -*-
"""Validated four-axis physical-chain kinematics.

This module contains the standard-DH chain used by the controller,
calibration solver and Cartesian motion interface.

Frame/convention
----------------
``base_to_j4`` returns ``T_base_j4`` (J4 frame expressed in the DH base frame).
The four standard-DH angles are the *total* angles used by each DH row; there
is no separate O3 term.  Translation is returned in metres.

The tool is not centred on the planar arm.  The often-quoted 82.4 mm is the
axis component measured from a CAD marker, not from the standard ``d4=0`` DH
origin.  Projecting the assembly onto the common-normal DH origin gives a
59.4 mm pitch-axis component.  Ignoring it still produces an approximately
6 cm lateral task-space error, so the TCP is represented as a full 3-D vector.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


# Corrected SolidWorks/CAD parallel-axis distances, validated by the current
# multi-point touch and commanded holdout tests.
A1_MM = -1.824
A2_MM = 227.452
A3_MM = 224.984

# Motor readings at the mechanically upright reference and the corresponding
# total standard-DH angles.  The J2/J3 signs were checked by real +/-3 deg moves.
MOTOR_REFERENCE_DEG = (-0.001, -0.104, -0.219, 0.001)
DH_REFERENCE_DEG = (0.0, 90.0, 0.0, 0.0)
MOTOR_SIGNS = (1.0, 1.0, -1.0, 1.0)

# J4-frame vector to the centre between the fingertip ends.  Local +X is the
# tool/fingertip direction and local +Z is the J4 pitch-axis direction.  This
# uses the standard d4=0 common-normal origin; it must not be replaced by the
# 82.383 mm axial component measured from the separate URDF/CAD marker p4.
CAD_TCP_J4_MM = np.array([198.133299, 0.484470, 59.404655], dtype=float)

# Real controller/config soft limits (motor angles, degrees).
DEFAULT_MOTOR_LIMITS = (
    (-160.0, 160.0),
    (-90.0, 129.0),
    (-144.0, 175.0),
    (-720.0, 720.0),
)


def _wrap180(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def motor_to_dh(motor_deg: Sequence[float]) -> List[float]:
    """Map J1..J4 motor readings to total standard-DH angles (degrees)."""
    if len(motor_deg) != 4:
        raise ValueError("expected four motor angles")
    return [
        DH_REFERENCE_DEG[i]
        + MOTOR_SIGNS[i] * (float(motor_deg[i]) - MOTOR_REFERENCE_DEG[i])
        for i in range(4)
    ]


def dh_to_motor(dh_deg: Sequence[float]) -> List[float]:
    """Inverse of :func:`motor_to_dh`."""
    if len(dh_deg) != 4:
        raise ValueError("expected four DH angles")
    return [
        MOTOR_REFERENCE_DEG[i]
        + MOTOR_SIGNS[i] * (float(dh_deg[i]) - DH_REFERENCE_DEG[i])
        for i in range(4)
    ]


def _dh(theta_deg: float, a_mm: float, alpha_deg: float = 0.0) -> np.ndarray:
    """Standard DH row with d=0; translation remains in millimetres."""
    th = math.radians(float(theta_deg))
    al = math.radians(float(alpha_deg))
    ct, st = math.cos(th), math.sin(th)
    ca, sa = math.cos(al), math.sin(al)
    return np.array([
        [ct, -st * ca, st * sa, float(a_mm) * ct],
        [st, ct * ca, -ct * sa, float(a_mm) * st],
        [0.0, sa, ca, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def base_to_j4(motor_deg: Sequence[float]) -> np.ndarray:
    """Return ``T_base_j4`` for motor readings; translation is in metres."""
    q1, q2, q3, q4 = motor_to_dh(motor_deg)
    transform = (
        _dh(q1, A1_MM, 90.0)
        @ _dh(q2, A2_MM)
        @ _dh(q3, A3_MM)
        @ _dh(q4, 0.0)
    )
    transform[:3, 3] /= 1000.0
    return transform


def joint_origins_base_mm(motor_deg: Sequence[float]) -> np.ndarray:
    """Return the four successive DH-frame origins in base coordinates (mm)."""
    q1, q2, q3, q4 = motor_to_dh(motor_deg)
    rows = (
        _dh(q1, A1_MM, 90.0),
        _dh(q2, A2_MM),
        _dh(q3, A3_MM),
        _dh(q4, 0.0),
    )
    transform = np.eye(4)
    origins = []
    for row in rows:
        transform = transform @ row
        origins.append(transform[:3, 3].copy())
    return np.asarray(origins, dtype=float)


def tcp_base_mm(
    motor_deg: Sequence[float],
    tcp_j4_mm: Sequence[float] = CAD_TCP_J4_MM,
) -> np.ndarray:
    """Return the TCP position in the DH base frame (millimetres)."""
    transform = base_to_j4(motor_deg)
    tcp = np.asarray(tcp_j4_mm, dtype=float).reshape(3) / 1000.0
    return (transform[:3, :3] @ tcp + transform[:3, 3]) * 1000.0


def tool_pitch_deg(motor_deg: Sequence[float]) -> float:
    """Tool +X pitch in the arm plane; +90 up and -90 down."""
    return float(sum(motor_to_dh(motor_deg)[1:]))


def within_motor_limits(
    motor_deg: Sequence[float],
    limits: Sequence[Tuple[float, float]] = DEFAULT_MOTOR_LIMITS,
) -> bool:
    return len(motor_deg) == 4 and all(
        float(limits[i][0]) <= float(motor_deg[i]) <= float(limits[i][1])
        for i in range(4)
    )


def ik_tcp_all(
    xyz_base_mm: Sequence[float],
    pitch_deg: float,
    *,
    tcp_j4_mm: Sequence[float] = CAD_TCP_J4_MM,
    motor_limits: Sequence[Tuple[float, float]] = DEFAULT_MOTOR_LIMITS,
    tolerance_mm: float = 1e-5,
) -> List[List[float]]:
    """Solve the four-axis task ``(x, y, z, pitch)`` for all valid branches.

    A full 6-D tool orientation is not controllable by this four-axis arm.
    ``pitch_deg`` fixes the tool +X direction in the arm plane.  The non-zero
    J4-axis component of ``tcp_j4_mm`` is handled analytically and therefore
    also changes the required J1 yaw.
    """
    x, y, z = np.asarray(xyz_base_mm, dtype=float).reshape(3)
    lx, ly, lz = np.asarray(tcp_j4_mm, dtype=float).reshape(3)
    rho = math.hypot(x, y)
    if rho + 1e-9 < abs(lz):
        return []

    signed_radial_abs = math.sqrt(max(0.0, rho * rho - lz * lz))
    pitch = math.radians(float(pitch_deg))
    tool_radial = lx * math.cos(pitch) - ly * math.sin(pitch)
    tool_vertical = lx * math.sin(pitch) + ly * math.cos(pitch)
    candidates: List[List[float]] = []

    for signed_tcp_radial in (signed_radial_abs, -signed_radial_abs):
        # [x,y] = Rz(q1) * [signed_tcp_radial, -lz].
        q1 = _wrap180(math.degrees(
            math.atan2(y, x) - math.atan2(-lz, signed_tcp_radial)))
        wrist_r = signed_tcp_radial - tool_radial - A1_MM
        wrist_z = z - tool_vertical
        cosine_elbow = (
            wrist_r * wrist_r + wrist_z * wrist_z - A2_MM * A2_MM - A3_MM * A3_MM
        ) / (2.0 * A2_MM * A3_MM)
        if abs(cosine_elbow) > 1.0 + 1e-9:
            continue
        cosine_elbow = max(-1.0, min(1.0, cosine_elbow))
        sine_abs = math.sqrt(max(0.0, 1.0 - cosine_elbow * cosine_elbow))

        for sine_elbow in (sine_abs, -sine_abs):
            q3 = math.atan2(sine_elbow, cosine_elbow)
            q2 = (
                math.atan2(wrist_z, wrist_r)
                - math.atan2(A3_MM * sine_elbow,
                             A2_MM + A3_MM * cosine_elbow)
            )
            q4 = pitch - q2 - q3
            dh_angles = [q1, math.degrees(q2), math.degrees(q3), math.degrees(q4)]
            motor = dh_to_motor(dh_angles)
            if not within_motor_limits(motor, motor_limits):
                continue
            if np.linalg.norm(tcp_base_mm(motor, tcp_j4_mm) - [x, y, z]) > tolerance_mm:
                continue
            candidates.append(motor)
    return candidates


def ik_tcp(
    xyz_base_mm: Sequence[float],
    pitch_deg: float,
    *,
    current_motor_deg: Sequence[float] | None = None,
    tcp_j4_mm: Sequence[float] = CAD_TCP_J4_MM,
    motor_limits: Sequence[Tuple[float, float]] = DEFAULT_MOTOR_LIMITS,
) -> List[float] | None:
    """Return the valid IK branch closest to ``current_motor_deg``."""
    candidates = ik_tcp_all(
        xyz_base_mm,
        pitch_deg,
        tcp_j4_mm=tcp_j4_mm,
        motor_limits=motor_limits,
    )
    if not candidates:
        return None
    if current_motor_deg is None:
        return candidates[0]
    current = np.asarray(current_motor_deg, dtype=float).reshape(4)

    def score(motor: Iterable[float]) -> float:
        delta = np.asarray(list(motor), dtype=float) - current
        delta[0] = _wrap180(delta[0])
        # Prefer the nearby elbow branch; J4 is less collision-critical.
        return float(abs(delta[0]) + 1.2 * abs(delta[1])
                     + 1.2 * abs(delta[2]) + 0.3 * abs(delta[3]))

    return min(candidates, key=score)
