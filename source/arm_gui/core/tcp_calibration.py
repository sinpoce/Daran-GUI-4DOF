# -*- coding: utf-8 -*-
"""Independent fixed-point TCP calibration for the four-axis arm.

Only the frozen J1--J4 forward kinematics enters this solver.  Camera poses,
target geometry and an existing TCP must never be used to build the equations.

For contact pose ``i``::

    p_ref = o_i + R_i @ t_tcp

``o_i`` and ``R_i`` are the J4 origin and orientation in the base frame.  The
unknowns are the three-dimensional J4-frame TCP vector and the (unmeasured)
fixed reference point.  A single radial branch of this particular four-axis
arm is rank deficient: changing pitch cannot observe the local J4 +Z TCP
component.  Full 3-D results are therefore rejected unless the centred
rotation system has rank three and passes explicit singular-value gates.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class PivotPose:
    """One independent physical contact, not one telemetry frame."""

    rotation_base_j4: np.ndarray
    origin_base_mm: np.ndarray
    label: str = ""

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation_base_j4, dtype=float)
        origin = np.asarray(self.origin_base_mm, dtype=float)
        if rotation.shape != (3, 3) or origin.shape != (3,):
            raise ValueError("pose requires a 3x3 rotation and 3-vector origin")
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(origin)):
            raise ValueError("pose contains non-finite values")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("pose rotation is not orthonormal")
        if np.linalg.det(rotation) < 0.999:
            raise ValueError("pose rotation is not right handed")
        object.__setattr__(self, "rotation_base_j4", rotation.copy())
        object.__setattr__(self, "origin_base_mm", origin.copy())


def pose_from_transform(transform_base_j4: Sequence[Sequence[float]],
                        label: str = "") -> PivotPose:
    """Create a pose from ``T_base_j4`` whose translation is in metres."""
    transform = np.asarray(transform_base_j4, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("T_base_j4 must be 4x4")
    return PivotPose(transform[:3, :3], transform[:3, 3] * 1000.0, label)


def _centred_rotation_system(poses: Sequence[PivotPose]) -> np.ndarray:
    rotations = np.asarray([pose.rotation_base_j4 for pose in poses], float)
    return (rotations - rotations.mean(axis=0)).reshape(-1, 3)


def observability(poses: Sequence[PivotPose]) -> dict:
    """Return scale-free TCP observability metrics.

    Singular values are computed from the centred rotation matrix divided by
    ``sqrt(N)``.  Full 3-D TCP estimation requires rank three.  With only one
    radial/J1 branch this arm should correctly report rank two.
    """
    if len(poses) < 2:
        return {
            "rank": 0,
            "singular_values": [0.0, 0.0, 0.0],
            "condition": None,
        }
    system = _centred_rotation_system(poses) / np.sqrt(float(len(poses)))
    singular = np.linalg.svd(system, compute_uv=False)
    singular = np.pad(singular, (0, max(0, 3 - len(singular))))[:3]
    tolerance = max(system.shape) * np.finfo(float).eps * max(singular[0], 1.0)
    rank = int(np.count_nonzero(singular > tolerance))
    condition = (float(singular[0] / singular[2])
                 if rank == 3 and singular[2] > 0.0 else None)
    return {
        "rank": rank,
        "singular_values": [float(value) for value in singular],
        "condition": condition,
    }


def _linear_system(poses: Sequence[PivotPose], fixed_local_z_mm: float | None
                   ) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    rhs = []
    identity = np.eye(3)
    for pose in poses:
        rotation = pose.rotation_base_j4
        if fixed_local_z_mm is None:
            rows.append(np.column_stack((rotation, -identity)))
            rhs.append(-pose.origin_base_mm)
        else:
            rows.append(np.column_stack((rotation[:, :2], -identity)))
            rhs.append(-pose.origin_base_mm
                       - rotation[:, 2] * float(fixed_local_z_mm))
    return np.vstack(rows), np.concatenate(rhs)


def _solve_weighted(poses: Sequence[PivotPose], weights: np.ndarray,
                    fixed_local_z_mm: float | None) -> tuple[np.ndarray, np.ndarray]:
    matrix, rhs = _linear_system(poses, fixed_local_z_mm)
    root_weights = np.repeat(np.sqrt(np.asarray(weights, float)), 3)
    solution, _, _, _ = np.linalg.lstsq(
        matrix * root_weights[:, None], rhs * root_weights, rcond=None)
    if fixed_local_z_mm is None:
        tcp = solution[:3]
        reference = solution[3:6]
    else:
        tcp = np.array([solution[0], solution[1], float(fixed_local_z_mm)])
        reference = solution[2:5]
    return tcp, reference


def _residual_vectors(poses: Sequence[PivotPose], tcp: np.ndarray,
                      reference: np.ndarray) -> np.ndarray:
    return np.asarray([
        pose.origin_base_mm + pose.rotation_base_j4 @ tcp - reference
        for pose in poses
    ], dtype=float)


def fit_pivot(
    poses: Iterable[PivotPose],
    *,
    fixed_local_z_mm: float | None = None,
    robust_iterations: int = 12,
    huber_delta_mm: float = 1.5,
) -> dict:
    """Fit a fixed-point TCP with contact-block Huber IRLS.

    ``fixed_local_z_mm=None`` requests a complete 3-D fit and refuses a
    rank-deficient pose set.  Supplying a local-Z value performs the explicitly
    constrained two-component alternative; the result records that constraint
    and must not be described as a fully independent 3-D TCP calibration.
    """
    pose_list = list(poses)
    unknown_tcp_components = 3 if fixed_local_z_mm is None else 2
    minimum = 3 if fixed_local_z_mm is None else 2
    if len(pose_list) < minimum:
        raise ValueError("need at least %d independent contacts" % minimum)

    obs = observability(pose_list)
    required_rank = unknown_tcp_components
    if obs["rank"] < required_rank:
        raise ValueError(
            "TCP pose set is unobservable: centred rotation rank %d, need %d"
            % (obs["rank"], required_rank))

    weights = np.ones(len(pose_list), dtype=float)
    tcp = np.zeros(3, dtype=float)
    reference = np.zeros(3, dtype=float)
    for _ in range(max(1, int(robust_iterations))):
        tcp, reference = _solve_weighted(
            pose_list, weights, fixed_local_z_mm)
        residual = _residual_vectors(pose_list, tcp, reference)
        norms = np.linalg.norm(residual, axis=1)
        median = float(np.median(norms))
        mad = float(np.median(np.abs(norms - median)))
        robust_scale = max(1.4826 * mad, 0.25)
        threshold = max(float(huber_delta_mm), 1.345 * robust_scale)
        new_weights = np.ones_like(weights)
        large = norms > threshold
        new_weights[large] = threshold / np.maximum(norms[large], 1e-12)
        if np.max(np.abs(new_weights - weights)) < 1e-7:
            weights = new_weights
            break
        weights = new_weights

    tcp, reference = _solve_weighted(pose_list, weights, fixed_local_z_mm)
    residual = _residual_vectors(pose_list, tcp, reference)
    norms = np.linalg.norm(residual, axis=1)
    median = float(np.median(norms))
    mad = float(np.median(np.abs(norms - median)))
    outlier_threshold = max(3.0 * 1.4826 * mad, 2.5)
    outliers = norms > outlier_threshold
    return {
        "method": ("fixed_point_full_3d"
                   if fixed_local_z_mm is None
                   else "fixed_point_xy_with_fixed_local_z"),
        "contact_count": len(pose_list),
        "tcp_j4_mm": [float(value) for value in tcp],
        "reference_base_mm": [float(value) for value in reference],
        "fixed_local_z_mm": (None if fixed_local_z_mm is None
                             else float(fixed_local_z_mm)),
        "observability": obs,
        "residual_vectors_mm": residual.tolist(),
        "residual_norms_mm": norms.tolist(),
        "rms_mm": float(np.sqrt(np.mean(norms ** 2))),
        "median_mm": median,
        "p90_mm": float(np.percentile(norms, 90)),
        "max_mm": float(np.max(norms)),
        "huber_weights": weights.tolist(),
        "outlier_threshold_mm": float(outlier_threshold),
        "outlier_indices": np.flatnonzero(outliers).astype(int).tolist(),
    }


def leave_one_out(poses: Iterable[PivotPose], *,
                  fixed_local_z_mm: float | None = None) -> dict:
    """Report TCP sensitivity when each independent contact is omitted."""
    pose_list = list(poses)
    full = fit_pivot(pose_list, fixed_local_z_mm=fixed_local_z_mm)
    baseline = np.asarray(full["tcp_j4_mm"], dtype=float)
    estimates = []
    skipped = []
    for index in range(len(pose_list)):
        subset = pose_list[:index] + pose_list[index + 1:]
        try:
            estimate = fit_pivot(subset, fixed_local_z_mm=fixed_local_z_mm)
        except ValueError as exc:
            skipped.append({"index": index, "reason": str(exc)})
            continue
        tcp = np.asarray(estimate["tcp_j4_mm"], dtype=float)
        estimates.append({
            "index": index,
            "tcp_j4_mm": tcp.tolist(),
            "delta_norm_mm": float(np.linalg.norm(tcp - baseline)),
        })
    deltas = [row["delta_norm_mm"] for row in estimates]
    return {
        "estimates": estimates,
        "skipped": skipped,
        "max_tcp_delta_mm": max(deltas) if deltas else None,
        "rms_tcp_delta_mm": (float(np.sqrt(np.mean(np.square(deltas))))
                             if deltas else None),
    }
