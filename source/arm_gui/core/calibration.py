# -*- coding: utf-8 -*-
"""Runtime support for the official multi-plane hand-eye calibration.

This module is deliberately independent from ``tools/`` so the same detector,
RGB-D/IMU capture and anchor-fusion solver work in both a source checkout and
the packaged GUI.
"""
from __future__ import annotations

import math
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from . import anchor_fusion
from . import kinematics
from . import target3d


# The union is intentional.  J1/J4 supply rotation diversity; the coupled
# J2/J3 configurations make camera translation observable on this four-axis
# arm.  Either subset alone can have excellent residuals while leaving the
# hand-eye translation almost free (metre-scale false solutions).
FULL_GRID: List[Tuple[float, float, float, float]] = [
    (0, 0, 0, 0),
    (0, 0, 0, 6), (0, 0, 0, 12), (0, 0, 0, 18),
    (0, 0, 0, -6), (0, 0, 0, -12), (0, 0, 0, -18),
    (-10, 0, 0, 0), (-20, 0, 0, 0), (-30, 0, 0, 0),
    (-20, 0, 0, 12), (-20, 0, 0, -12),
    (10, 0, 0, 0), (20, 0, 0, 0), (30, 0, 0, 0),
    (20, 0, 0, 12), (20, 0, 0, -12), (30, 0, 0, -6),
]

COUPLED_GRID: List[Tuple[float, float, float, float]] = [
    (0, 0, 0, 0),
    (0, 6, 0, -6), (10, 6, 0, -6), (-10, 6, 0, -6),
    (0, 6, 0, -6), (0, 0, 0, 0),
    (0, -6, 0, 6), (-10, -6, 0, 6), (10, -6, 0, 6),
    (0, -6, 0, 6), (0, 0, 0, 0),
    (0, 0, 4, 4), (10, 0, 4, 4), (-10, 0, 4, 4),
    (0, 0, 4, 4), (0, 0, 0, 0),
    (0, 0, -5, -5), (-10, 0, -5, -5), (10, 0, -5, -5),
    (0, 0, -5, -5), (0, 0, 0, 0),
    (0, 5, 5, 0), (10, 5, 5, 0), (-10, 5, 5, 0),
    (0, 5, 5, 0), (0, 0, 0, 0),
    (0, -5, -5, 0), (-10, -5, -5, 0), (10, -5, -5, 0),
    (0, -5, -5, 0), (0, 0, 0, 0),
]

CALIBRATION_GRID: List[Tuple[float, float, float, float]] = (
    FULL_GRID + [(0, 0, 0, 0)] + COUPLED_GRID[1:]
)

MIN_PREFLIGHT_CORNERS = 36
MIN_CAPTURE_CORNERS = 20
MIN_DEPTH_LEVELS = 2
MAX_REG_RMS_MM = 3.0
MIN_SPREAD_MM = 20.0


def _intrinsics_metadata(intrinsics) -> Dict:
    '''Return a JSON-friendly description of a RealSense video model.'''
    return {
        'width': int(intrinsics.width),
        'height': int(intrinsics.height),
        'fx': float(intrinsics.fx),
        'fy': float(intrinsics.fy),
        'ppx': float(intrinsics.ppx),
        'ppy': float(intrinsics.ppy),
        'distortion_model': str(intrinsics.model),
        'coeffs': [float(value) for value in intrinsics.coeffs],
    }


def _extrinsics_metadata(extrinsics) -> Dict:
    '''Return RealSense extrinsics using metres for translation.'''
    rotation = np.asarray(extrinsics.rotation, dtype=float).reshape(
        3, 3, order='F')
    return {
        'rotation': rotation.tolist(),
        'translation_m': [float(value) for value in extrinsics.translation],
    }


def _stream_metadata(profile) -> Dict:
    '''Describe either a video or a motion stream profile.'''
    row = {
        'stream_type': str(profile.stream_type()),
        'format': str(profile.format()),
        'fps': int(profile.fps()),
        'stream_index': int(profile.stream_index()),
        'unique_id': int(profile.unique_id()),
    }
    try:
        video = profile.as_video_stream_profile()
        row.update(width=int(video.width()), height=int(video.height()))
    except Exception:
        pass
    return row


def _frame_metadata(frame, host_monotonic: float) -> Dict:
    '''Record both device-clock identity and the host receipt time.'''
    return {
        'frame_number': int(frame.get_frame_number()),
        'device_timestamp_ms': float(frame.get_timestamp()),
        'timestamp_domain': str(frame.get_frame_timestamp_domain()),
        'host_monotonic': float(host_monotonic),
        'host_time_unix': float(time.time()),
    }


class CalibrationCamera:
    """One D435i color/depth/IMU session owned by the calibration worker."""

    def __init__(self) -> None:
        self.pipe = None
        self.align = None
        self.profile = None
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        self.depth_scale = 0.0
        self.R_color_imu: Optional[np.ndarray] = None

    def open(self) -> Dict:
        import pyrealsense2 as rs

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.accel)
        cfg.enable_stream(rs.stream.gyro)
        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        device = self.profile.get_device()
        self.depth_scale = float(
            device.first_depth_sensor().get_depth_scale())
        color_profile = self.profile.get_stream(
            rs.stream.color).as_video_stream_profile()
        depth_profile = self.profile.get_stream(
            rs.stream.depth).as_video_stream_profile()
        accel_profile = self.profile.get_stream(rs.stream.accel)
        gyro_profile = self.profile.get_stream(rs.stream.gyro)
        intr = color_profile.get_intrinsics()
        depth_intr = depth_profile.get_intrinsics()
        self.K = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        self.dist = np.asarray(intr.coeffs, dtype=float).reshape(-1)
        extr = accel_profile.get_extrinsics_to(color_profile)
        self.R_color_imu = np.asarray(
            extr.rotation, dtype=float).reshape(3, 3, order='F')
        depth_to_color = depth_profile.get_extrinsics_to(color_profile)

        def read_device_info(key) -> Optional[str]:
            try:
                if device.supports(key):
                    return str(device.get_info(key))
            except Exception:
                pass
            return None

        camera_name = read_device_info(rs.camera_info.name)
        serial_number = read_device_info(rs.camera_info.serial_number)
        firmware_version = read_device_info(rs.camera_info.firmware_version)
        sdk_version = str(getattr(rs, '__version__', 'unknown'))
        stream_config = {
            'color': _stream_metadata(color_profile),
            'depth': _stream_metadata(depth_profile),
            'accel': _stream_metadata(accel_profile),
            'gyro': _stream_metadata(gyro_profile),
        }
        camera = {
            'name': camera_name,
            'serial_number': serial_number,
            'firmware_version': firmware_version,
            'sdk_version': sdk_version,
        }
        return {
            'K': self.K.copy(),
            'dist': self.dist.copy(),
            'R_color_imu': self.R_color_imu.copy(),
            'depth_scale': self.depth_scale,
            'depth_scale_m_per_unit': self.depth_scale,
            'camera': camera,
            'camera_name': camera_name,
            'serial': serial_number,
            'serial_number': serial_number,
            'firmware': firmware_version,
            'firmware_version': firmware_version,
            'sdk_version': sdk_version,
            'color_intrinsics': _intrinsics_metadata(intr),
            'depth_intrinsics': _intrinsics_metadata(depth_intr),
            'depth_to_color_extrinsics': _extrinsics_metadata(
                depth_to_color),
            'accel_to_color_extrinsics': _extrinsics_metadata(extr),
            'stream_config': stream_config,
            'model_path': target3d.default_model_path(),
        }

    def grab_packet(self, timeout_ms: int = 1000) -> Optional[Dict]:
        """Return one aligned video packet plus any IMU samples in the frameset."""
        if self.pipe is None or self.align is None:
            return None
        import pyrealsense2 as rs

        frames = self.pipe.wait_for_frames(int(timeout_ms))
        accel: List[List[float]] = []
        gyro: List[List[float]] = []
        accel_samples: List[Dict] = []
        gyro_samples: List[Dict] = []
        for frame in frames:
            if not frame.is_motion_frame():
                continue
            value = frame.as_motion_frame().get_motion_data()
            row = [float(value.x), float(value.y), float(value.z)]
            sample = {'xyz': row}
            sample.update(_frame_metadata(frame, time.monotonic()))
            stream_type = frame.get_profile().stream_type()
            if stream_type == rs.stream.accel:
                accel.append(row)
                accel_samples.append(sample)
            elif stream_type == rs.stream.gyro:
                gyro.append(row)
                gyro_samples.append(sample)
        aligned = self.align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            return None
        host_monotonic = time.monotonic()
        bgr = np.asanyarray(color.get_data()).copy()
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        depth_raw = np.asanyarray(depth.get_data()).copy()
        if depth_raw.dtype != np.uint16:
            depth_raw = depth_raw.astype(np.uint16)
        depth_m = depth_raw.astype(np.float32) * self.depth_scale
        color_frame = _frame_metadata(color, host_monotonic)
        depth_frame = _frame_metadata(depth, host_monotonic)
        return {
            'timestamp': host_monotonic,
            'host_monotonic': host_monotonic,
            'host_time_unix': float(time.time()),
            'bgr': bgr,
            'gray': gray,
            'depth_raw': depth_raw,
            'depth_raw16': depth_raw,
            'depth_m': depth_m,
            'depth_aligned_to': 'color',
            'alignment': {
                'depth_aligned_to': 'color',
                'color_shape': [int(bgr.shape[0]), int(bgr.shape[1])],
                'depth_shape': [int(depth_raw.shape[0]),
                                int(depth_raw.shape[1])],
            },
            'color_frame': color_frame,
            'depth_frame': depth_frame,
            'color_frame_number': color_frame['frame_number'],
            'color_device_timestamp_ms':
                color_frame['device_timestamp_ms'],
            'color_timestamp_domain': color_frame['timestamp_domain'],
            'depth_frame_number': depth_frame['frame_number'],
            'depth_device_timestamp_ms':
                depth_frame['device_timestamp_ms'],
            'depth_timestamp_domain': depth_frame['timestamp_domain'],
            'accel': accel,
            'gyro': gyro,
            'accel_samples': accel_samples,
            'gyro_samples': gyro_samples,
            'imu_samples': {
                'accel': accel_samples,
                'gyro': gyro_samples,
            },
        }

    def close(self) -> None:
        if self.pipe is not None:
            try:
                self.pipe.stop()
            except Exception:
                pass
        self.pipe = None
        self.profile = None
        self.align = None


def draw_overlay(gray: np.ndarray,
                 corners: Dict[int, np.ndarray]) -> np.ndarray:
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    for gid, point in corners.items():
        q = tuple(np.round(point).astype(int))
        cv2.circle(image, q, 4, (0, 210, 0), -1)
        cv2.putText(image, str(gid), (q[0] + 4, q[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 230, 255), 1, cv2.LINE_AA)
    return image


def probe_frame(gray: np.ndarray, depth_m: np.ndarray,
                K: np.ndarray, dist: np.ndarray,
                model: target3d.TargetModel) -> Dict:
    corners = target3d.detect_corners(gray)
    points, plane_info = target3d.corners_3d_from_depth(
        corners, depth_m, K, dist, model)
    _transform, registration = target3d.register_depth_to_model(
        points, model) if points else (None, {'ok': False, 'why': 'no points'})
    ok_levels = [int(level) for level, info in plane_info.items()
                 if info.get('ok')]
    passed = bool(
        len(corners) >= MIN_PREFLIGHT_CORNERS
        and len(ok_levels) == 3
        and registration.get('ok')
        and registration.get('rms_mm', 99.0) <= MAX_REG_RMS_MM
        and registration.get('spread_min_mm', 0.0) >= MIN_SPREAD_MM)
    return {
        'ok': passed,
        'corners': corners,
        'n_corners': len(corners),
        'points_cam_m': points,
        'n_depth_points': len(points),
        'plane_info': plane_info,
        'ok_levels': ok_levels,
        'registration': registration,
    }


def build_sample(grays: Sequence[np.ndarray], depths_m: Sequence[np.ndarray],
                 accel_rows: Sequence[Sequence[float]], motors: Sequence[float],
                 K: np.ndarray, dist: np.ndarray, R_color_imu: np.ndarray,
                 model: target3d.TargetModel) -> Tuple[Optional[Dict], Dict]:
    """Convert a short static burst into one fusion sample and diagnostics."""
    corners, pixel_std, counts = target3d.detect_corners_multi(
        grays, min_seen=min(3, len(grays)))
    depth_med = (np.median(np.stack(depths_m), axis=0)
                 if depths_m else None)
    points: Dict[int, np.ndarray] = {}
    plane_info: Dict = {}
    if depth_med is not None and corners:
        points, plane_info = target3d.corners_3d_from_depth(
            corners, depth_med, K, dist, model)
    _transform, registration = target3d.register_depth_to_model(
        points, model) if points else (None, {'ok': False, 'why': 'no points'})
    ok_levels = [int(level) for level, info in plane_info.items()
                 if info.get('ok')]
    accepted = bool(
        len(corners) >= MIN_CAPTURE_CORNERS
        and len(ok_levels) >= MIN_DEPTH_LEVELS
        and registration.get('ok')
        and registration.get('rms_mm', 99.0) <= MAX_REG_RMS_MM
        and registration.get('spread_min_mm', 0.0) >= MIN_SPREAD_MM)

    accel = np.asarray(accel_rows, dtype=float).reshape(-1, 3)
    accel_mean = accel.mean(axis=0) if len(accel) else None
    f_color = None
    if accel_mean is not None:
        vector = np.asarray(R_color_imu, float).reshape(3, 3) @ accel_mean
        norm = float(np.linalg.norm(vector))
        if norm > 1e-9:
            f_color = vector / norm

    T_bg = kinematics.base_to_gripper(motors)
    sample = None
    if accepted:
        sample = {
            'T_bg': T_bg,
            'uv_px': {int(g): np.asarray(p, float) for g, p in corners.items()},
            'pts_cam_m': {int(g): np.asarray(p, float)
                          for g, p in points.items()},
        }
        if f_color is not None:
            sample['f'] = f_color
            sample['a_raw'] = accel_mean

    quality = {
        'accepted': accepted,
        'n_corners': len(corners),
        'n_depth_points': len(points),
        'ok_levels': ok_levels,
        'registration': registration,
        'plane_info': plane_info,
        'pixel_std_max': max(
            (float(np.linalg.norm(value)) for value in pixel_std.values()),
            default=None),
        'detection_counts_min': min(counts.values()) if counts else 0,
        'n_accel': int(len(accel)),
        'accel_mean': accel_mean.tolist() if accel_mean is not None else None,
        'gravity_color': f_color.tolist() if f_color is not None else None,
        'motor': [float(value) for value in motors],
        'T_bg': T_bg.tolist(),
        'global_corner_ids': [int(g) for g in sorted(corners)],
        'corners_px': [corners[g].tolist() for g in sorted(corners)],
        'corners_xyz_cam_m': [
            points[g].tolist() if g in points else None for g in sorted(corners)
        ],
    }
    return sample, quality


def _result_failure(message: str, n: int = 0) -> Dict:
    return {
        'ok': False, 'method': 'ANCHOR_FUSION_3D', 'n': int(n),
        'R': None, 't': None, 'X': None, 'T_base_target': None,
        'reproj_rms_px': None, 'depth_rms_mm': None,
        'gravity_rms_deg': None, 'numeric_gates': {}, 'msg': message,
        'promotion_allowed': False,
    }


def solve_samples(samples: Sequence[Dict], K: np.ndarray, dist: np.ndarray,
                  model: target3d.TargetModel) -> Dict:
    """Run depth, gravity and joint fusion with explicit physical gates."""
    rows = list(samples)
    if len(rows) < 12:
        return _result_failure(
            'Need at least 12 accepted poses (got %d).' % len(rows), len(rows))

    model_points = {int(gid): point for gid, point in model.pts_mm.items()}
    gravity_pairs = []
    for sample in rows:
        if sample.get('f') is None:
            continue
        T_bg = np.asarray(sample['T_bg'], float)
        u = T_bg[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        gravity_pairs.append({
            'u': u,
            'f': np.asarray(sample['f'], float),
            'a_raw': sample.get('a_raw'),
        })
    if len(gravity_pairs) < 8:
        return _result_failure(
            'Need at least 8 poses with IMU gravity (got %d).' %
            len(gravity_pairs), len(rows))

    try:
        gravity = anchor_fusion.solve_gravity(gravity_pairs)
        depth = anchor_fusion.solve_depth(rows, model_points)
    except Exception as exc:
        return _result_failure('Anchor initialisation failed: %s' % exc,
                               len(rows))
    if not depth.get('ok'):
        return _result_failure('Depth initialisation failed: %s' %
                               depth.get('msg', ''), len(rows))

    # The merged trajectory makes the depth seed well-conditioned.  Retain an
    # automatic gravity-seeded retry and select by gate count so a rare bad
    # depth-axis initialisation cannot silently produce a metre-scale solution.
    candidates = []
    for label, kwargs in (
            ('depth_seed', {'X0': depth['X'], 'T_bt0': depth['T_bt']}),
            ('auto_seed', {})):
        try:
            fusion = anchor_fusion.solve_fusion(
                rows, model_points, K, dist,
                sigma_px=0.7, sigma_depth_m=0.004,
                sigma_grav_rad=0.002, estimate_bias=False, robust=False,
                **kwargs)
            candidates.append((label, fusion))
        except Exception:
            continue
    if not candidates:
        return _result_failure('Fusion solver failed to start.', len(rows))

    def assess(item):
        _label, fusion = item
        if fusion.get('X') is None:
            return -1, float('inf')
        X = np.asarray(fusion['X'], float)
        vs_gravity = anchor_fusion.rotation_angle_deg(
            X[:3, :3], gravity['R_x'])
        vs_depth = anchor_fusion.rotation_angle_deg(
            X[:3, :3], np.asarray(depth['X'])[:3, :3])
        norm_mm = float(np.linalg.norm(X[:3, 3]) * 1000.0)
        gates = [
            bool(fusion.get('success')),
            fusion.get('rms_px', 99.0) <= 1.2,
            fusion.get('rms_depth_mm', 99.0) <= 4.0,
            fusion.get('rms_grav_deg', 99.0) <= 0.3,
            vs_gravity <= 1.0,
            vs_depth <= 2.0,
            100.0 <= norm_mm <= 270.0,
            fusion.get('jacobian_cond', float('inf')) <= 1000.0,
        ]
        score = (float(fusion.get('rms_px', 99.0))
                 + float(fusion.get('rms_depth_mm', 99.0)) / 4.0
                 + float(fusion.get('rms_grav_deg', 99.0)) / 0.3)
        return sum(gates), score

    seed_name, fusion = sorted(
        candidates, key=lambda item: (-assess(item)[0], assess(item)[1]))[0]
    if fusion.get('X') is None:
        return _result_failure(fusion.get('msg', 'Fusion failed.'), len(rows))
    X = np.asarray(fusion['X'], float)
    T_bt = np.asarray(fusion['T_bt'], float)
    vs_gravity = anchor_fusion.rotation_angle_deg(
        X[:3, :3], gravity['R_x'])
    vs_depth = anchor_fusion.rotation_angle_deg(
        X[:3, :3], np.asarray(depth['X'])[:3, :3])
    norm_mm = float(np.linalg.norm(X[:3, 3]) * 1000.0)
    gates = {
        'sample_count': len(rows) >= 40,
        'fusion_success': bool(fusion.get('success')),
        'reprojection_rms': fusion.get('rms_px', 99.0) <= 1.2,
        'depth_rms': fusion.get('rms_depth_mm', 99.0) <= 4.0,
        'gravity_rms': fusion.get('rms_grav_deg', 99.0) <= 0.3,
        'fusion_vs_gravity': vs_gravity <= 1.0,
        'fusion_vs_depth': vs_depth <= 2.0,
        'camera_translation_sanity': 100.0 <= norm_mm <= 270.0,
        'jacobian_condition': fusion.get(
            'jacobian_cond', float('inf')) <= 1000.0,
    }
    numeric_pass = bool(all(gates.values()))
    if numeric_pass:
        message = ('Formal calibration passed all numeric gates. Target '
                   'geometry provenance is %s; absolute accuracy remains '
                   'limited by the manufactured target tolerance.'
                   % model.provenance())
    else:
        failed = ', '.join(name for name, passed in gates.items() if not passed)
        message = 'Calibration rejected by gate(s): %s.' % failed
    return {
        'ok': numeric_pass,
        'method': 'ANCHOR_FUSION_3D',
        'seed': seed_name,
        'n': len(rows),
        'R': X[:3, :3].tolist(),
        't': X[:3, 3].tolist(),
        'X': X.tolist(),
        'T_base_target': T_bt.tolist(),
        'reproj_rms_px': float(fusion['rms_px']),
        'depth_rms_mm': float(fusion['rms_depth_mm']),
        'gravity_rms_deg': float(fusion['rms_grav_deg']),
        'gravity_only_rms_deg': float(gravity['rms_deg']),
        'gravity_arc_span_deg': float(gravity['arc_span_deg']),
        'fusion_vs_gravity_deg': float(vs_gravity),
        'fusion_vs_depth_deg': float(vs_depth),
        'camera_translation_norm_mm': norm_mm,
        'jacobian_condition': float(fusion['jacobian_cond']),
        'depth_registration_rms_mm': float(depth['rms_mm']),
        'numeric_gates': gates,
        'model_provenance': model.provenance(),
        'formal_result': bool(numeric_pass),
        'geometry_measured': bool(model.is_measured),
        'promotion_allowed': bool(numeric_pass),
        'msg': message,
    }


def to_native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): to_native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_native(item) for item in value]
    return value
