# -*- coding: utf-8 -*-
"""三维多平面 ChArUco 靶的检测与三维对应生成。

把一帧 (彩色, 对齐深度) 变成求解器需要的东西：

    像素观测   {global_id: (u, v)}          → 视觉重投影残差用
    相机系3D点 {global_id: (x, y, z) 米}    → 深度锚 3D-3D 配准用
    靶标模型   {global_id: (x, y, z) 米}    → 上面两者的共同参照

要点（都是踩过坑换来的）：

* **分层检测**：三条带各自是一个独立 CharucoBoard，marker ID 分段
  (L0:0-11 / L1:12-23 / L2:24-35)，检测互不干扰；global_id = level*14 + local。
* **关联是索引恒等**：模型 JSON 里的 corners_target_mm 已经按 global_id 排好，
  且生成时已施加 v = STRIP_H - board_y 的手性映射。这里**绝不能再翻一次**。
* **深度不取单像素**：先用该层检出角点的凸包 (向内侵蚀) 圈出板面区域，
  RANSAC 拟合平面，每个角点取「彩色去畸变射线 ∩ 该平面」的交点。
  直接读角点处的原始深度会吃到 1-2mm 的边缘飞点，且棋盘格黑白边界正是
  深度最不可靠的地方。
* **深度必须用 K_color 反投影**：深度流已 rs.align(color)，落在彩色像面上。
  用 K_depth 是错的（早期 depth_poc 就是栽在这）。
"""
from __future__ import annotations

import json
import os
from functools import lru_cache
from typing import Dict, Optional, Tuple

import numpy as np
import cv2

N_LOCAL = 14                      # 每条带的内部角点数 (8x3 -> 7x2)
LEVEL_MARKERS = ((0, 11), (12, 23), (24, 35))


# --------------------------------------------------------------- 模型加载
class TargetModel:
    """正式标定板模型的只读视图；公开点坐标使用毫米。"""

    def __init__(self, path: str):
        with open(path, encoding='utf-8') as f:
            self.raw = json.load(f)
        self.path = path
        self.pts_mm: Dict[int, np.ndarray] = {}
        self.level_of: Dict[int, int] = {}
        for surf in self.raw['surfaces']:
            lv = int(surf['level'])
            gids = surf.get('global_corner_ids')
            corners = surf['corners_target_mm']
            if gids is None:
                gids = [lv * N_LOCAL + i for i in range(len(corners))]
            for gid, p in zip(gids, corners):
                self.pts_mm[int(gid)] = np.asarray(p, dtype=float)
                self.level_of[int(gid)] = lv
        mg = self.raw.get('measured_geometry')
        self.is_measured = bool(mg) and bool(mg.get('frozen'))
        if self.is_measured:
            # 实测几何优先（P3 冻结后）
            for gid, p in zip(mg['global_corner_ids'], mg['corners_measured_target_mm']):
                self.pts_mm[int(gid)] = np.asarray(p, dtype=float)

    @property
    def pts_m(self) -> Dict[int, np.ndarray]:
        return {g: p / 1000.0 for g, p in self.pts_mm.items()}

    def subset_m(self, gids) -> np.ndarray:
        return np.asarray([self.pts_mm[int(g)] for g in gids], dtype=float) / 1000.0

    def __len__(self):
        return len(self.pts_mm)

    def provenance(self) -> str:
        return 'measured(frozen)' if self.is_measured else 'design(CAD)'


def default_model_path() -> str:
    """靶标模型 JSON：随包只读资源（打包后在 _MEIPASS，回退到 exe 同级）。"""
    from arm_gui.core.apppaths import res_path
    return res_path('calibration', 'target', 'target_model.json')


# --------------------------------------------------------------- 检测
@lru_cache(maxsize=1)
def _boards():
    """Build the three official ChArUco strips without importing ``tools/``.

    The old implementation imported a generator from ``tools`` at runtime.
    That works from a source checkout but the ``tools`` directory is not part
    of the packaged GUI, so the calibration page could never detect the
    target after PyInstaller packaging. Keep the runtime detector entirely in
    ``core`` and derive the board dimensions from the shipped model metadata.
    """
    with open(default_model_path(), encoding='utf-8') as handle:
        raw = json.load(handle)
    design = raw['design']
    sx, sy = (int(v) for v in design['strip_squares'])
    square_m = float(design['square_mm']) / 1000.0
    marker_m = float(design['marker_mm']) / 1000.0
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, design['dict_name']))
    boards = []
    for surface in sorted(raw['surfaces'], key=lambda row: int(row['level'])):
        ids = np.asarray(surface['marker_ids'], dtype=np.int32)
        boards.append(cv2.aruco.CharucoBoard(
            (sx, sy), square_m, marker_m, dictionary, ids=ids))
    return boards, design


def detect_corners(gray: np.ndarray) -> Dict[int, np.ndarray]:
    """逐层检测，返回 {global_id: (u, v) 像素}。"""
    boards, _ = _boards()
    out: Dict[int, np.ndarray] = {}
    for lv, board in enumerate(boards):
        det = cv2.aruco.CharucoDetector(board)
        cc, ci, _mc, mi = det.detectBoard(gray)
        if cc is None or ci is None:
            continue
        lo, hi = LEVEL_MARKERS[lv]
        if mi is not None:
            seen = set(np.asarray(mi).ravel().tolist())
            if len(seen & set(range(lo, hi + 1))) < 4:
                continue           # 本层 marker 太少，判为未检出
        for pix, local in zip(np.asarray(cc).reshape(-1, 2),
                              np.asarray(ci).ravel().astype(int)):
            if 0 <= local < N_LOCAL:
                out[lv * N_LOCAL + int(local)] = np.asarray(pix, dtype=float)
    return out


def detect_corners_multi(grays, min_seen: int = 3) -> Tuple[Dict[int, np.ndarray],
                                                            Dict[int, np.ndarray],
                                                            Dict[int, int]]:
    """多帧检测后逐角点平均，抑制像素噪声。

    返回 (均值像素, 像素标准差, 每角点被检出的帧数)。只保留出现 >= min_seen 帧的角点
    ——偶发误检不会被平均掉，只会污染，必须靠出现次数筛掉。
    """
    acc: Dict[int, list] = {}
    for g in grays:
        for gid, pix in detect_corners(g).items():
            acc.setdefault(gid, []).append(pix)
    mean, std, count = {}, {}, {}
    for gid, lst in acc.items():
        if len(lst) < min_seen:
            continue
        a = np.asarray(lst, dtype=float)
        mean[gid] = a.mean(axis=0)
        std[gid] = a.std(axis=0)
        count[gid] = len(lst)
    return mean, std, count


# --------------------------------------------------------------- 深度 → 3D
def _fit_plane_ransac(pts: np.ndarray, thresh: float = 0.004,
                      iters: int = 200, rng_seed: int = 0):
    """返回 (法向 n, 平面上一点 c, 内点掩码)。n 归一化，未定向。"""
    if len(pts) < 3:
        return None, None, None
    rng = np.random.default_rng(rng_seed)
    best_mask, best_n, best_c = None, None, None
    best_cnt = -1
    for _ in range(iters):
        idx = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        d = np.abs((pts - p0) @ n)
        mask = d < thresh
        if mask.sum() > best_cnt:
            best_cnt, best_mask, best_n, best_c = mask.sum(), mask, n, p0
    if best_mask is None or best_cnt < 3:
        return None, None, None
    # 用全部内点做一次最小二乘精修
    q = pts[best_mask]
    c = q.mean(axis=0)
    _u, _s, vt = np.linalg.svd(q - c)
    return vt[2], c, best_mask


def corners_3d_from_depth(corners_px: Dict[int, np.ndarray],
                          depth_m: np.ndarray,
                          K_color: np.ndarray,
                          dist_color: np.ndarray,
                          model: TargetModel,
                          erode_px: int = 4,
                          plane_thresh_m: float = 0.004,
                          max_plane_rms_m: float = 0.006):
    """按层做「凸包分割 → RANSAC平面 → 射线∩平面」得到每角点的相机系 3D 点。

    depth_m: 已对齐到彩色的深度图，单位米，0 表示无效。
    返回 (pts_cam_m, info)。info 里带每层平面 RMS / 内点数 / 是否可用。
    """
    H, W = depth_m.shape[:2]
    out: Dict[int, np.ndarray] = {}
    info = {}

    by_level: Dict[int, list] = {}
    for gid in corners_px:
        by_level.setdefault(model.level_of[gid], []).append(gid)

    for lv, gids in sorted(by_level.items()):
        pix = np.asarray([corners_px[g] for g in gids], dtype=np.float32)
        if len(pix) < 4:
            info[lv] = {'ok': False, 'why': 'corners<4'}
            continue

        # 1) 板面掩膜：检出角点的凸包，向内侵蚀避开黑白边界与板边
        hull = cv2.convexHull(pix.reshape(-1, 1, 2)).astype(np.int32)
        mask = np.zeros((H, W), np.uint8)
        cv2.fillConvexPoly(mask, hull, 255)
        if erode_px > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                          (2 * erode_px + 1, 2 * erode_px + 1))
            mask = cv2.erode(mask, k)

        ys, xs = np.nonzero((mask > 0) & (depth_m > 0.05) & (depth_m < 2.0))
        if len(xs) < 50:
            info[lv] = {'ok': False, 'why': 'depth pixels<50', 'n': int(len(xs))}
            continue

        # 2) 反投影这些像素到相机系（务必用 K_color：深度已对齐到彩色像面）
        z = depth_m[ys, xs]
        fx, fy = K_color[0, 0], K_color[1, 1]
        cx, cy = K_color[0, 2], K_color[1, 2]
        P = np.stack([(xs - cx) / fx * z, (ys - cy) / fy * z, z], axis=1)

        # 3) RANSAC 平面
        n, c, inl = _fit_plane_ransac(P, thresh=plane_thresh_m)
        if n is None:
            info[lv] = {'ok': False, 'why': 'plane fit failed'}
            continue
        resid = (P[inl] - c) @ n
        rms = float(np.sqrt(np.mean(resid ** 2)))
        ok = rms <= max_plane_rms_m
        info[lv] = {'ok': bool(ok), 'plane_rms_m': rms,
                    'inliers': int(inl.sum()), 'n_px': int(len(xs)),
                    'normal_cam': n.tolist()}
        if not ok:
            continue

        # 4) 每个角点：去畸变射线 ∩ 平面（不取原始单像素深度）
        und = cv2.undistortPoints(pix.reshape(-1, 1, 2), K_color, dist_color)
        for gid, uv in zip(gids, und.reshape(-1, 2)):
            ray = np.array([uv[0], uv[1], 1.0])
            denom = float(ray @ n)
            if abs(denom) < 1e-6:
                continue
            t = float(c @ n) / denom          # 射线参数：解 (t*ray - c)·n = 0
            if 0.05 < t < 2.0:
                out[int(gid)] = ray * t

    return out, info


# --------------------------------------------------------------- 配准
def kabsch(src: np.ndarray, dst: np.ndarray):
    """求 R,t 使 R@src_i + t ≈ dst_i。返回 (R, t, rms, s_min)。

    s_min 是中心化 src 的最小奇异值：接近 0 说明点近似共线/共面，
    此时旋转在某个方向不可定 —— 必须报出来而不是闷头返回一个解。
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    if len(src) < 3:
        return None, None, float('nan'), 0.0
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    A, B = src - cs, dst - cd
    Hm = A.T @ B
    U, S, Vt = np.linalg.svd(Hm)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T      # det=+1，杜绝反射解
    t = cd - R @ cs
    rms = float(np.sqrt(np.mean(np.sum((src @ R.T + t - dst) ** 2, axis=1))))
    s_min = float(np.linalg.svd(A, compute_uv=False)[-1])
    return R, t, rms, s_min


def register_depth_to_model(pts_cam_m: Dict[int, np.ndarray], model: TargetModel):
    """3D-3D 配准得到 T_cam_target（相机系 <- 靶标系）。"""
    gids = sorted(pts_cam_m.keys())
    if len(gids) < 3:
        return None, {'ok': False, 'why': 'pts<3', 'n': len(gids)}
    src = model.subset_m(gids)                       # 靶标系 (米)
    dst = np.asarray([pts_cam_m[g] for g in gids])   # 相机系 (米)
    R, t, rms, s_min = kabsch(src, dst)
    if R is None:
        return None, {'ok': False, 'why': 'kabsch failed'}
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    levels = sorted({model.level_of[g] for g in gids})
    return T, {'ok': True, 'rms_mm': rms * 1000.0, 'n': len(gids),
               'levels': levels, 'spread_min_mm': s_min * 1000.0,
               'n_levels': len(levels)}
