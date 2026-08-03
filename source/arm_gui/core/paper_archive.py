# -*- coding: utf-8 -*-
from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import tempfile
import uuid
from datetime import datetime
from typing import Dict, Optional, Sequence

import cv2
import numpy as np

from .apppaths import user_path


def _native(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return _native(value.item())
    if isinstance(value, dict):
        return {str(key): _native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_native(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.tmp_', dir=os.path.dirname(path))
    try:
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _write_json(path: str, payload) -> None:
    text = json.dumps(_native(payload), ensure_ascii=False, indent=2) + '\n'
    _atomic_write(path, text.encode('utf-8'))


def _write_png(path: str, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode('.png', np.ascontiguousarray(image))
    if not ok:
        raise IOError('OpenCV could not encode %s' % path)
    _atomic_write(path, encoded.tobytes())


def _copy_snapshot(source: Optional[str], destination: str) -> Optional[Dict]:
    if not source or not os.path.isfile(source):
        return None
    shutil.copy2(source, destination)
    return {'path': os.path.abspath(destination),
            'sha256': sha256_file(destination),
            'bytes': os.path.getsize(destination)}


class PaperCalibrationArchive:
    '''One immutable-on-completion paper evidence session.'''

    def __init__(self, root: str, run_id: str, manifest: Dict) -> None:
        self.root = root
        self.run_id = run_id
        self.poses_dir = os.path.join(root, 'poses')
        self.snapshots_dir = os.path.join(root, 'snapshots')
        self.manifest = manifest
        self.pose_count = 0
        self.finalized = False

    @classmethod
    def create(cls, camera_info: Dict, home: Sequence[float],
               grid: Sequence[Sequence[float]], model_path: str,
               model_provenance: str, config_snapshot: Optional[Dict] = None,
               base_dir: Optional[str] = None):
        base = os.path.abspath(base_dir or user_path('paper', 'data'))
        os.makedirs(base, exist_ok=True)
        stamp = datetime.now().astimezone().strftime('%Y%m%dT%H%M%S_%f')[:-3]
        for _attempt in range(100):
            run_id = 'calibration_%s_%s' % (stamp, uuid.uuid4().hex[:8])
            session_root = os.path.join(base, run_id)
            try:
                os.mkdir(session_root)
                break
            except FileExistsError:
                continue
        else:
            raise IOError('Could not allocate a unique paper run directory.')
        root = os.path.join(session_root, 'calibration')
        os.mkdir(root)
        os.mkdir(os.path.join(root, 'poses'))
        os.mkdir(os.path.join(root, 'snapshots'))
        manifest = {
            'schema': 'paper-calibration-archive-v1',
            'run_id': run_id,
            'session_root': os.path.abspath(session_root),
            'created_at': datetime.now().astimezone().isoformat(
                timespec='milliseconds'),
            'status': 'COLLECTING',
            'expected_poses': len(grid),
            'saved_poses': [],
            'camera': _native(camera_info),
            'home_motor_deg': _native(home),
            'calibration_grid_deg': _native(grid),
            'target_model_provenance': model_provenance,
            'snapshots': {},
        }
        archive = cls(root, run_id, manifest)
        archive._snapshot_inputs(model_path, config_snapshot)
        archive._write_manifest()
        return archive

    def _write_manifest(self) -> None:
        _write_json(os.path.join(self.root, 'manifest.json'), self.manifest)

    def _snapshot_inputs(self, model_path: str,
                         config_snapshot: Optional[Dict]) -> None:
        copies = (
            ('target_model', model_path, 'target_model.json'),
            ('measured_geometry',
             user_path('calibration', 'measured_target_geometry.json'),
             'measured_target_geometry.json'),
            ('formal_result_before',
             user_path('calibration', 'handeye_result.yaml'),
             'formal_result_before.yaml'),
        )
        for label, source, name in copies:
            copied = _copy_snapshot(
                source, os.path.join(self.snapshots_dir, name))
            if copied:
                self.manifest['snapshots'][label] = copied
        if config_snapshot is None:
            return
        cfg_path = os.path.join(self.snapshots_dir, 'config_snapshot.json')
        _write_json(cfg_path, config_snapshot)
        self.manifest['snapshots']['config'] = {
            'path': os.path.abspath(cfg_path),
            'sha256': sha256_file(cfg_path),
            'bytes': os.path.getsize(cfg_path),
        }

    @staticmethod
    def _overlay(gray: np.ndarray, quality: Dict) -> np.ndarray:
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        ids = quality.get('global_corner_ids') or []
        corners = quality.get('corners_px') or []
        for gid, point in zip(ids, corners):
            q = tuple(np.rint(np.asarray(point, float)).astype(int))
            cv2.circle(image, q, 4, (0, 210, 0), -1)
            cv2.putText(image, str(gid), (q[0] + 4, q[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1,
                        cv2.LINE_AA)
        return image

    @staticmethod
    def _depth_colormap(depth_raw: np.ndarray) -> np.ndarray:
        valid = depth_raw[depth_raw > 0]
        if valid.size == 0:
            scaled = np.zeros(depth_raw.shape, dtype=np.uint8)
        else:
            low, high = np.percentile(valid, [2.0, 98.0])
            span = max(float(high - low), 1.0)
            scaled = np.clip((depth_raw.astype(float) - low) * 255.0 / span,
                             0.0, 255.0).astype(np.uint8)
            scaled[depth_raw == 0] = 0
        return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)

    def _save_solver_burst(self, pose_dir: str,
                           packets: Sequence[Dict]) -> Dict:
        selected = list(packets[-8:])
        if len(selected) < 8:
            raise ValueError('Eight paired RGB-D frames are required.')
        burst_dir = os.path.join(pose_dir, 'solver_burst')
        os.mkdir(burst_dir)
        metadata = []
        files = {}
        for offset, packet in enumerate(selected):
            bgr = packet.get('bgr')
            raw = packet.get('depth_raw16', packet.get('depth_raw'))
            alignment = packet.get('alignment') or {}
            if (bgr is None or raw is None
                    or (packet.get('depth_aligned_to') != 'color'
                        and alignment.get('depth_aligned_to') != 'color')):
                raise ValueError('Solver burst lacks genuine aligned RGB-D.')
            gray = np.asarray(packet['gray'], dtype=np.uint8)
            bgr = np.asarray(bgr, dtype=np.uint8)
            raw = np.asarray(raw, dtype=np.uint16)
            if bgr.shape[:2] != gray.shape or raw.shape != gray.shape:
                raise ValueError('Solver burst frame dimensions do not match.')
            prefix = 'frame_%02d' % offset
            rows = {
                prefix + '_color.png': bgr,
                prefix + '_gray.png': gray,
                prefix + '_depth_raw16.png': raw,
            }
            for name, image in rows.items():
                path = os.path.join(burst_dir, name)
                _write_png(path, image)
                relative = os.path.join('solver_burst', name)
                files[relative] = {
                    'sha256': sha256_file(path),
                    'bytes': os.path.getsize(path),
                }
            depth_path = os.path.join(
                burst_dir, prefix + '_depth_raw16.png')
            checked = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if (checked is None or checked.dtype != np.uint16
                    or not np.array_equal(checked, raw)):
                raise IOError('Solver-burst depth is not lossless.')
            metadata.append({
                'offset': offset,
                'used_for_gray_solver': offset >= 3,
                'used_for_depth_solver': True,
                'host_monotonic': packet.get('host_monotonic'),
                'host_time_unix': packet.get('host_time_unix'),
                'color_frame': packet.get('color_frame'),
                'depth_frame': packet.get('depth_frame'),
            })
        manifest_path = os.path.join(burst_dir, 'burst_manifest.json')
        _write_json(manifest_path, {
            'paired_frame_count': len(selected),
            'gray_solver_frames': 5,
            'depth_solver_frames': 8,
            'frames': metadata,
        })
        relative = os.path.join('solver_burst', 'burst_manifest.json')
        files[relative] = {
            'sha256': sha256_file(manifest_path),
            'bytes': os.path.getsize(manifest_path),
        }
        return files

    def save_pose(self, index: int, packets: Sequence[Dict], quality: Dict,
                  robot_state: Dict, target_motor: Sequence[float],
                  delta_motor: Sequence[float], depth_scale: float) -> str:
        if self.finalized:
            raise RuntimeError('Cannot add a pose to a finalized archive.')
        if not packets:
            raise ValueError('No RGB-D packet was captured for this pose.')
        pose_dir = os.path.join(self.poses_dir, 'pose_%03d' % (index + 1))
        os.mkdir(pose_dir)
        packet = packets[-1]
        gray = np.asarray(packet['gray'], dtype=np.uint8)
        bgr_value = packet.get('bgr')
        if bgr_value is None:
            raise ValueError('A genuine color frame is required.')
        bgr = np.asarray(bgr_value, dtype=np.uint8)
        raw_value = packet.get('depth_raw16', packet.get('depth_raw'))
        if raw_value is None:
            raise ValueError('A genuine aligned uint16 depth frame is required.')
        alignment = packet.get('alignment') or {}
        if (packet.get('depth_aligned_to') != 'color'
                and alignment.get('depth_aligned_to') != 'color'):
            raise ValueError('Depth packet is not declared aligned to color.')
        depth_raw = np.asarray(raw_value, dtype=np.uint16)
        if bgr.shape[:2] != gray.shape or depth_raw.shape != gray.shape:
            raise ValueError('Color and aligned depth dimensions do not match.')
        images = {
            'color.png': bgr,
            'gray.png': gray,
            'depth_raw16.png': depth_raw,
            'depth_colormap.png': self._depth_colormap(depth_raw),
            'overlay.png': self._overlay(gray, quality),
        }
        for name, image in images.items():
            _write_png(os.path.join(pose_dir, name), image)
        checked = cv2.imread(
            os.path.join(pose_dir, 'depth_raw16.png'), cv2.IMREAD_UNCHANGED)
        if (checked is None or checked.dtype != np.uint16
                or not np.array_equal(checked, depth_raw)):
            raise IOError('Saved 16-bit depth failed lossless verification.')
        burst_files = self._save_solver_burst(pose_dir, packets)
        buffer = io.BytesIO()
        np.save(buffer, depth_raw, allow_pickle=False)
        _atomic_write(os.path.join(pose_dir, 'depth_raw16.npy'),
                      buffer.getvalue())
        accel = []
        gyro = []
        timestamps = []
        for row in packets:
            accel.extend(row.get('accel_samples') or [])
            gyro.extend(row.get('gyro_samples') or [])
            timestamps.append({
                'host_monotonic': row.get('host_monotonic',
                                          row.get('timestamp')),
                'host_time_unix': row.get('host_time_unix'),
                'color_frame': row.get('color_frame'),
                'depth_frame': row.get('depth_frame'),
                'alignment': row.get('alignment'),
            })
        if not accel or not gyro:
            raise ValueError(
                'Structured accel and gyro evidence are both required.')
        _write_json(os.path.join(pose_dir, 'imu_window.json'), {
            'accel': accel, 'gyro': gyro,
        })
        _write_json(os.path.join(pose_dir, 'timestamps.json'), timestamps)
        _write_json(os.path.join(pose_dir, 'robot_state.json'), robot_state)
        _write_json(os.path.join(pose_dir, 'quality.json'), quality)
        record = {
            'pose_index': int(index),
            'directory': os.path.abspath(pose_dir),
            'accepted': bool(quality.get('accepted')),
            'target_motor_deg': _native(target_motor),
            'delta_motor_deg': _native(delta_motor),
            'depth_scale_m_per_unit': float(depth_scale),
            'depth_aligned_to': 'color',
            'packet_count': len(packets),
            'accel_sample_count': len(accel),
            'gyro_sample_count': len(gyro),
            'raw_camera_evidence': True,
            'files': {},
        }
        for name in (list(images) + ['depth_raw16.npy', 'imu_window.json',
                                     'timestamps.json', 'robot_state.json',
                                     'quality.json']):
            path = os.path.join(pose_dir, name)
            record['files'][name] = {
                'sha256': sha256_file(path),
                'bytes': os.path.getsize(path),
            }
        record['files'].update(burst_files)
        _write_json(os.path.join(pose_dir, 'pose_manifest.json'), record)
        self.pose_count += 1
        self.manifest['saved_poses'].append(record)
        self._write_manifest()
        return pose_dir

    def finalize(self, status: str, result: Optional[Dict], header: Dict,
                 quality_rows: Sequence[Dict], reason: str = '') -> str:
        if self.finalized:
            return os.path.join(self.root, 'calibration_run.json')
        status = str(status).upper()
        if status not in ('PASS', 'REJECTED', 'ABORTED', 'ERROR'):
            raise ValueError('Unknown archive status: %s' % status)
        run_path = os.path.join(self.root, 'calibration_run.json')
        _write_json(run_path, {
            'run_header': header,
            'poses': list(quality_rows),
            'result': result,
            'status': status,
            'reason': reason,
        })
        self.manifest.update({
            'status': status,
            'finished_at': datetime.now().astimezone().isoformat(
                timespec='milliseconds'),
            'pose_count': self.pose_count,
            'accepted_poses': sum(
                1 for row in quality_rows if row.get('accepted')),
            'complete_pose_archive': (
                self.pose_count == self.manifest['expected_poses']),
            'reason': reason,
            'calibration_run': {
                'path': os.path.abspath(run_path),
                'sha256': sha256_file(run_path),
                'bytes': os.path.getsize(run_path),
            },
        })
        self.finalized = True
        self._write_manifest()
        return run_path

    def _verify_integrity(self) -> bool:
        expected = int(self.manifest.get('expected_poses', -1))
        records = self.manifest.get('saved_poses') or []
        if self.pose_count != expected or len(records) != expected:
            return False
        if sorted(int(row.get('pose_index', -1)) for row in records) != list(
                range(expected)):
            return False
        required = {
            'color.png', 'gray.png', 'depth_raw16.png',
            'depth_colormap.png', 'overlay.png', 'depth_raw16.npy',
            'imu_window.json', 'timestamps.json', 'robot_state.json',
            'quality.json',
        }
        required.add(os.path.join('solver_burst', 'burst_manifest.json'))
        for offset in range(8):
            prefix = os.path.join('solver_burst', 'frame_%02d_' % offset)
            required.update({prefix + 'color.png', prefix + 'gray.png',
                             prefix + 'depth_raw16.png'})
        pose_root = os.path.abspath(self.poses_dir) + os.sep
        for record in records:
            if (not record.get('raw_camera_evidence')
                    or int(record.get('packet_count', 0)) < 8
                    or int(record.get('accel_sample_count', 0)) < 1
                    or int(record.get('gyro_sample_count', 0)) < 1):
                return False
            directory = os.path.abspath(record.get('directory') or '')
            if not directory.startswith(pose_root):
                return False
            files = record.get('files') or {}
            if not required.issubset(files):
                return False
            for name, metadata in files.items():
                path = os.path.abspath(os.path.join(directory, name))
                if (not path.startswith(directory + os.sep)
                        or not os.path.isfile(path)
                        or sha256_file(path) != metadata.get('sha256')
                        or os.path.getsize(path) != metadata.get('bytes')):
                    return False
        for label in ('target_model', 'config'):
            metadata = (self.manifest.get('snapshots') or {}).get(label) or {}
            path = metadata.get('path')
            if (not path or not os.path.isfile(path)
                    or sha256_file(path) != metadata.get('sha256')):
                return False
        run_meta = self.manifest.get('calibration_run') or {}
        run_path = run_meta.get('path')
        if (not run_path or not os.path.isfile(run_path)
                or sha256_file(run_path) != run_meta.get('sha256')):
            return False
        return True

    def should_promote(self, result: Optional[Dict]) -> bool:
        gates = (result or {}).get('numeric_gates') or {}
        return bool(
            self.finalized
            and self.manifest.get('status') == 'PASS'
            and self.manifest.get('complete_pose_archive')
            and self._verify_integrity()
            and result and result.get('ok')
            and result.get('promotion_allowed')
            and gates and all(bool(value) for value in gates.values()))

    def promote_formal(self, result: Dict, header: Dict,
                       formal_root: Optional[str] = None) -> str:
        import yaml

        run_path = os.path.join(self.root, 'calibration_run.json')
        with open(run_path, encoding='utf-8') as handle:
            archived = json.load(handle)
        archived_result = archived.get('result') or {}
        archived_header = archived.get('run_header') or {}
        if (_native(result) != archived_result
                or _native(header) != archived_header):
            raise RuntimeError(
                'Promotion inputs differ from the finalized run archive.')
        if not self.should_promote(archived_result):
            raise RuntimeError('Result or paper archive is not promotable.')
        result = archived_result
        header = archived_header

        root = os.path.abspath(formal_root or user_path('calibration'))
        versions = os.path.join(root, 'formal_results')
        os.makedirs(versions, exist_ok=True)
        active = os.path.join(root, 'handeye_result.yaml')
        if os.path.isfile(active):
            old_hash = sha256_file(active)
            old_copy = os.path.join(
                versions, 'prior_%s.yaml' % old_hash[:16])
            if not os.path.exists(old_copy):
                shutil.copy2(active, old_copy)
        payload = {'result': result, 'meta': header,
                   'raw_run': os.path.abspath(run_path),
                   'paper_archive': os.path.abspath(self.root)}
        encoded = yaml.safe_dump(
            _native(payload), allow_unicode=True,
            sort_keys=False).encode('utf-8')
        session_result = os.path.join(self.root, 'handeye_result.yaml')
        _atomic_write(session_result, encoded)
        version = os.path.join(
            versions, 'handeye_%s.yaml' % self.run_id)
        _atomic_write(version, encoded)
        result_hash = hashlib.sha256(encoded).hexdigest()
        self.manifest['formal_promotion'] = {
            'active_path': os.path.abspath(active),
            'version_path': os.path.abspath(version),
            'session_path': os.path.abspath(session_result),
            'sha256': result_hash,
            'activation': 'atomic active replacement is the final operation',
            'promoted_at': datetime.now().astimezone().isoformat(
                timespec='milliseconds'),
        }
        self._write_manifest()
        _atomic_write(active, encoded)
        return active
"""Lossless archive."""
