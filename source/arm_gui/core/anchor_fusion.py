# -*- coding: utf-8 -*-
"""多锚手眼融合求解器（重力 Wahba + 深度 Kabsch + 加权最小二乘融合）。

由 build-anchor-solvers 工作流的三个独立实现合并而来，并应用了对抗性复核的修正。
关键约定（与 config_registry.json 冻结值一致，勿改符号）：
  X = T_g_c（相机在抓手系），平移单位米
  u_i = R_base_gripper^T · [0,0,1]；f_i = 单位比力(+up) 在 COLOR 系
  配对 R_x^T u_i = f_i
模型点文件是毫米，模块内部一律米。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
from scipy.optimize import least_squares

_GRAV_NORM_EPS = 1e-6       # input direction vectors below this norm are invalid
_GRAV_PARALLEL_TOL = 1e-4   # s2/s1 of B below this => u_i set effectively parallel
_GRAV_MIN_N_FOR_BIAS = 5    # [复核 D2] bias 需要足够残差自由度，否则 rms 被拟合到 0


def _grav_skew(v):
    x, y, z = np.asarray(v, float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _grav_unit(v, name: str) -> np.ndarray:
    """Defensive unit-normalisation; near-zero vectors indicate an upstream bug."""
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < _GRAV_NORM_EPS:
        raise ValueError('solve_gravity: %s has near-zero norm (%.3g)' % (name, n))
    return v / n


def _grav_wahba(us: List[np.ndarray], fs: List[np.ndarray]):
    """det-corrected Kabsch: R maximising sum_i u_i^T R f_i  (so R f_i ~= u_i).

    Returns (R, singular_values_of_B). The diag(1,1,sign(det)) correction is
    load-bearing here, not hygiene: J1 (yaw about base z) never changes u_i
    (Rz(d) commutes with [0,0,1]), so the u_i sweep only the J4 pitch arc
    (<= +/-18 deg) and B = sum u f^T is numerically rank 2. Rank-deficient
    Kabsch without the correction can return a reflection (det = -1).
    """
    B = np.zeros((3, 3))
    for u, f in zip(us, fs):
        B += np.outer(u, f)
    U, S, Vt = np.linalg.svd(B)
    d = float(np.linalg.det(U @ Vt))
    corr = np.diag([1.0, 1.0, 1.0 if d >= 0.0 else -1.0])
    return U @ corr @ Vt, S


def solve_gravity(pairs: List[Dict], estimate_bias: bool = False) -> Dict:
    """Hand-eye ROTATION from the gravity anchor (closed-form Wahba/Kabsch).

    pairs : list of dicts
        {'u': (3,) FK-predicted gravity direction in the gripper frame,
         'f': (3,) measured specific-force direction (+up) in the COLOR frame,
         'a_raw': (3,) OPTIONAL unnormalised mean accel (m/s^2, COLOR frame) —
                  required per pair only when estimate_bias=True}.
    estimate_bias : refine a constant accelerometer bias b (m/s^2) jointly with
        R_x via block-coordinate alternation. A bias is UNRECOVERABLE from unit
        vectors alone (normalisation discards magnitude), so 'a_raw' is
        mandatory for it; missing a_raw => bias=None with an explanatory 'msg',
        never a silent point estimate.

    Returns dict:
        'R_x'          (3,3) rotation block of X = T_g_c satisfying R_x^T u ~= f
        'residual_deg' per-pair angle(f_i, R_x^T u_i) in degrees
        'rms_deg'      sqrt(mean(residual_deg^2))
        'bias'         (3,) m/s^2 or None
        'n'            number of pairs used
      diagnostics:
        'singular_values'  of B = sum u f^T — s2/s1 IS the conditioning of the
                           yaw-about-gravity component of R_x
        'arc_span_deg'     max pairwise angle within the u_i set
        'degenerate_yaw'   True when n < 2 or the u_i are effectively parallel:
                           R_x is then a 1-DOF family (any rotation about the
                           common gravity axis fits equally well — the
                           DOF-match argument; yaw must come from other anchors)
        'bias_singular_values' / 'bias_condition' / 'bias_weak_axis'
                           conditioning of the stacked bias Jacobian (only when
                           a bias was estimated): with a J4-only pitch arc one
                           bias axis is near-inseparable from a yaw tweak of
                           R_x — expect one near-zero singular value; that axis
                           is NOT measured.
        'msg'              human-readable notes.
    """
    if not pairs:
        raise ValueError('solve_gravity: empty pair list')

    us, fs, a_raws = [], [], []
    for k, p in enumerate(pairs):
        us.append(_grav_unit(p['u'], "pairs[%d]['u']" % k))
        fs.append(_grav_unit(p['f'], "pairs[%d]['f']" % k))
        # 'a_raw' 与 'a' 都接受：两个键名在不同调用方各自沿用，宽进严出。
        a = p.get('a_raw', p.get('a', None))
        a_raws.append(None if a is None else np.asarray(a, dtype=float).reshape(3))
    n = len(us)

    msgs = []
    bias = None
    bias_sv = None
    bias_cond = None
    bias_weak_axis = None

    do_bias = bool(estimate_bias)
    if do_bias and any(a is None for a in a_raws):
        # 显式要求估 bias 却没给原始加速度：单位向量丢掉了模长，bias 在原理上
        # 不可辨识。这是调用错误，必须报错——静默返回 None 会被漏看。
        raise ValueError(
            "solve_gravity(estimate_bias=True) requires raw accel per pair "
            "(key 'a_raw' or 'a', m/s^2, COLOR frame); a constant bias is "
            "unrecoverable from unit direction vectors alone")
    # [复核 D2] 自由度门：Wahba 吃掉 3 个、bias 再吃 3 个自由度。n<5 时 bias 会
    # 把残差拟合到 0，rms_deg 这个"物理质量指标"随之失去意义（实测 n=2 时
    # rms 0.1195°->0.0000° 而 R_x 误差不变）。宁可不给 bias，也不给假 rms。
    if do_bias and n < _GRAV_MIN_N_FOR_BIAS:
        msgs.append('estimate_bias needs n>=%d for residual DOF (got %d); '
                    'bias would drive rms to a meaningless ~0 -> bias=None'
                    % (_GRAV_MIN_N_FOR_BIAS, n))
        do_bias = False

    if do_bias:
        # Alternation: (1) fixed b -> closed-form Wahba on f_i(b); (2) fixed R
        # -> one Gauss-Newton step on b. Coupling is weak (b moves f_i on the
        # sphere, R re-aligns), so this converges in a handful of iterations.
        b = np.zeros(3)
        J = np.zeros((3 * n, 3))
        for _ in range(15):
            fb = []
            for i, a in enumerate(a_raws):
                v = a - b
                nv = float(np.linalg.norm(v))
                if nv < _GRAV_NORM_EPS:
                    raise ValueError('solve_gravity: a_raw[%d] - bias is '
                                     'near-zero; bad accel data or diverged '
                                     'bias' % i)
                fb.append(v / nv)
            R, _ = _grav_wahba(us, fb)
            rows, errs = [], []
            for i, a in enumerate(a_raws):
                v = a - b
                nv = float(np.linalg.norm(v))
                f_i = v / nv
                # d f / d b = -(I - f f^T)/|a-b| : tangent-projected because a
                # radial shift of (a - b) cannot change a unit vector — this is
                # exactly why magnitudes (a_raw) are needed for bias at all.
                rows.append(-(np.eye(3) - np.outer(f_i, f_i)) / nv)
                errs.append(f_i - R.T @ us[i])
            J = np.vstack(rows)                      # (3n, 3)
            e = np.concatenate(errs)
            # lstsq with a truncation threshold: the (near-)null bias axis gets
            # no update instead of a wild one.
            db, _res, _rank, _sv = np.linalg.lstsq(J, -e, rcond=1e-8)
            b = b + db
            if float(np.linalg.norm(db)) < 1e-10:
                break
        bias = b
        # [复核 D1] 关键修正：只看 b 的雅可比(固定 R)会给出错误的可观测性结论。
        # 真正的简并存在于**联合** (R, b) 问题里：在 ±18° 的俯仰弧上，一个切向
        # bias 有 1-cos(18°)≈97.5% 可被旋转微调吸收，即 bias 直接变成 R_x 的倾斜
        # (倾角 = b_tan/g)，而残差反而更小。必须用联合 3n×6 雅可比判条件数。
        R_cur, _ = _grav_wahba(us, [(a - b) / float(np.linalg.norm(a - b))
                                    for a in a_raws])
        J_joint = np.zeros((3 * n, 6))
        for i, a in enumerate(a_raws):
            v = a - b
            nv = float(np.linalg.norm(v))
            f_i = v / nv
            J_joint[3 * i:3 * i + 3, 0:3] = _grav_skew(R_cur.T @ us[i])
            J_joint[3 * i:3 * i + 3, 3:6] = -(np.eye(3) - np.outer(f_i, f_i)) / nv
        _uj, sj, vjt = np.linalg.svd(J_joint, full_matrices=False)
        bias_sv = sj
        bias_cond = float(sj[0] / max(float(sj[-1]), 1e-300))
        # 近零右奇异向量的 b 分量 = 真正测不到的 bias 方向
        bias_weak_axis = vjt[-1][3:6].copy()
        if float(np.linalg.norm(bias_weak_axis)) > 0.5:
            msgs.append('bias 有一个方向与 R_x 微调近简并(联合条件数 %.1f)；'
                        '该方向的 bias 未被测到，会 1:1 变成 R_x 倾斜 '
                        '(倾角≈b_tan/g)。必须用不含IMU的锚(如深度)交叉核验。'
                        % bias_cond)
        # Residuals below are evaluated against the bias-corrected directions.
        fs = [(a - b) / float(np.linalg.norm(a - b)) for a in a_raws]

    R_x, S = _grav_wahba(us, fs)

    residual_deg = []
    for i in range(n):
        c = float(np.clip(np.dot(fs[i], R_x.T @ us[i]), -1.0, 1.0))
        residual_deg.append(math.degrees(math.acos(c)))
    rms_deg = float(np.sqrt(np.mean(np.square(residual_deg))))

    arc_span_deg = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            c = float(np.clip(np.dot(us[i], us[j]), -1.0, 1.0))
            arc_span_deg = max(arc_span_deg, math.degrees(math.acos(c)))

    s2_over_s1 = float(S[1] / S[0]) if S[0] > 0.0 else 0.0
    degenerate_yaw = bool(n < 2 or s2_over_s1 < _GRAV_PARALLEL_TOL)
    # [复核 D3] s2/s1 ∝ tan²(span/2)，是 span 的二次量，不能当作 yaw 不确定度。
    # 布尔量只在"几乎完全平行"时才触发，会让 2–8° 张角的弱条件情形蒙混过关。
    # 给一个可直接判读的定量值：yaw_sigma ≈ rms / (sin(span/2)·√n)。
    _half = math.sin(math.radians(arc_span_deg) / 2.0)
    yaw_sigma_deg = (rms_deg / (_half * math.sqrt(n))
                     if _half > 1e-9 and n > 0 else float('inf'))
    if degenerate_yaw:
        msgs.append('degenerate: <2 non-parallel gravity directions '
                    '(s2/s1=%.3g, arc span %.3f deg) — R_x is a 1-DOF family '
                    'about the gravity axis; returned the Kabsch minimiser '
                    'but its yaw component is arbitrary' % (s2_over_s1,
                                                            arc_span_deg))

    out = {
        'R_x': R_x,
        'residual_deg': residual_deg,
        'rms_deg': rms_deg,
        'bias': bias,
        'n': n,
        'singular_values': S.tolist(),
        'arc_span_deg': arc_span_deg,
        'degenerate_yaw': degenerate_yaw,
        'yaw_sigma_deg': yaw_sigma_deg,
        'msg': '; '.join(msgs) if msgs else
               'gravity Wahba: %d pairs, rms %.4f deg' % (n, rms_deg),
    }
    # 诊断键**恒定存在**（无 bias 时为 None）：调用方按 key 取值不该因分支而 KeyError，
    # 也不该拿到"键不存在"和"值为None"两种含义相同却写法不同的结果。
    out['bias_singular_values'] = (bias_sv.tolist() if bias_sv is not None else None)
    out['bias_condition'] = bias_cond
    out['bias_weak_axis'] = bias_weak_axis
    return out


def rotation_angle_deg(Ra, Rb) -> float:
    """Geodesic (axis-angle) distance between two rotations, degrees."""
    Ra = np.asarray(Ra, dtype=float)
    Rb = np.asarray(Rb, dtype=float)
    c = (float(np.trace(Ra.T @ Rb)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def rotation_angle_about_axis_deg(Ra, Rb, axis) -> float:
    """Signed component of the Ra->Rb disagreement about `axis`, degrees.

    Frame convention (pinned): the relative rotation is taken on the LEFT,
        R_rel = Rb · Ra^T          (i.e. Rb = R_rel · Ra),
    so for hand-eye rotations R_x (p_grip = R_x p_cam) both R_rel and `axis`
    live in the GRIPPER frame. The headline pitch metric uses
    axis = [0,0,1]: the DH chain's A4 is a pure Rz, so the J4 pitch axis is
    gripper-z. Returns degrees(rodrigues(R_rel) · unit(axis)) — a signed
    projection, not an absolute value.
    """
    Ra = np.asarray(Ra, dtype=float)
    Rb = np.asarray(Rb, dtype=float)
    r_rel = Rb @ Ra.T
    rvec = cv2.Rodrigues(np.ascontiguousarray(r_rel))[0].reshape(3)
    a = np.asarray(axis, dtype=float).reshape(3)
    na = float(np.linalg.norm(a))
    if na < 1e-12:
        raise ValueError('rotation_angle_about_axis_deg: zero axis')
    return math.degrees(float(np.dot(rvec, a / na)))

_DEPTH_COLLINEAR_S_RATIO = 1e-3
# If the Kabsch det-correction fires while H is far from rank-2 (s3/s1 large),
# the point sets are genuinely reflected -> the id association itself is wrong.
_DEPTH_DET_FIRE_S3_RATIO = 0.1
# Relative rotations below this angle have a numerically undefined axis.
_DEPTH_AXIS_MIN_ANGLE_DEG = 1.0


def _depth_proper_rot(b: np.ndarray) -> np.ndarray:
    """argmax_R tr(R^T B) over SO(3): R = U diag(1,1,det(UV^T)) V^T, B = USV^T.

    The det-correction is load-bearing, not hygiene: the depth block routinely
    feeds rank-2 matrices (coplanar target strip -> rank-2 H; two-axis motion
    -> rank-2 axis bundle), where an uncorrected SVD can return a reflection.
    """
    u, _s, vt = np.linalg.svd(b)
    d = float(np.sign(np.linalg.det(u @ vt)))
    return u @ np.diag([1.0, 1.0, d]) @ vt


def _depth_rot_angle_deg(r: np.ndarray) -> float:
    """Geodesic angle (deg) of a single rotation matrix (local diagnostic use)."""
    c = (float(np.trace(r)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def solve_depth(samples: List[Dict], model_pts_mm: Dict, min_pts: int = 3) -> Dict:
    """Depth anchor: per-pose 3D-3D Kabsch registration, then one (X, T_bt).

    samples : list of dicts {'T_bg': (4,4) base<-gripper (metres),
              'pts_cam_m': {global_id: (3,) xyz in camera COLOR frame, metres}}.
    model_pts_mm : dict {global_id: (3,) target-frame point, MILLIMETRES}
              (converted to metres exactly once, here at ingest).

    Per pose: Kabsch model->camera gives T_cam_target (no PnP, no intrinsics,
    no planar-flip ambiguity: 3D-3D with known per-point correspondence and
    det=+1 is unique even for a coplanar target). Then the single consistent
    X = T_g_c and T_bt are recovered across ALL poses from the chain identity
    T_bt = T_bg(i) . X . T_ct(i):
      * rotation R_x by axis-Wahba on the AX=XB pair axes (closed form; honest
        but deliberately weak along the J4 direction with this arm's motion --
        conditioning is reported so callers can prefer the gravity anchor),
      * R_bt as the chordal mean of R_bg(i) R_x R_ct(i),
      * (t_x, t_bt) jointly from a linear 3Nx6 stack (never from pose 0 alone).

    Returns dict with keys 'X' (4,4), 'T_bt' (4,4), 'per_pose_rms_mm', 'rms_mm',
    'n_poses', plus diagnostics: 'ok', 'T_ct_list', 'pose_indices', 'skipped',
    'per_pose_model_min_sval', 'id_association_flags', 'R_x_source',
    'axis_singular_values', 'n_axis_pairs', 'translation_cond',
    'consistency_rms_mm', 'consistency_rot_rms_deg', 'msg'.
    On failure 'ok' is False, 'X'/'T_bt' are None, and 'msg' says why.
    """
    min_pts = max(int(min_pts), 3)                 # Kabsch needs >=3 non-collinear
    # mm -> metres exactly once (the single unit conversion of this block)
    model_m = {int(g): np.asarray(p, float).reshape(3) / 1000.0
               for g, p in model_pts_mm.items()}

    # ---- step 1: per-pose 3D-3D Kabsch -> T_ct(i) ----
    poses: List[Dict] = []
    skipped: List[Dict] = []
    for idx, s in enumerate(samples):
        pts = s.get('pts_cam_m') or {}
        gids = sorted(int(g) for g in pts if int(g) in model_m)
        if len(gids) < min_pts:
            skipped.append({'index': idx, 'reason': 'too_few_points',
                            'n_common': len(gids), 'model_min_sval': None})
            continue
        a = np.array([model_m[g] for g in gids], dtype=float)                # target, m
        b = np.array([np.asarray(pts[g], float).reshape(3) for g in gids])   # camera, m
        a_c = a - a.mean(axis=0)
        b_c = b - b.mean(axis=0)

        # Collinearity guard on the MODEL configuration (it, not the data noise,
        # decides whether the rotation is determined). Coplanar (s3 ~ 0) is fine.
        sv_model = np.linalg.svd(a_c, compute_uv=False)
        if sv_model[0] <= 0.0 or sv_model[1] / sv_model[0] < _DEPTH_COLLINEAR_S_RATIO:
            skipped.append({'index': idx, 'reason': 'collinear_model_points',
                            'n_common': len(gids),
                            'model_min_sval': float(sv_model[-1])})
            continue

        h = a_c.T @ b_c                            # 3x3 cross-covariance
        u, sh, vt = np.linalg.svd(h)
        det_fire = float(np.linalg.det(vt.T @ u.T)) < 0.0
        r_ct = vt.T @ np.diag([1.0, 1.0, -1.0 if det_fire else 1.0]) @ u.T
        t_ct = b.mean(axis=0) - r_ct @ a.mean(axis=0)

        # det-correction firing on a well-conditioned H (s3 not small) means the
        # two point sets really are mirror images -> the id association is wrong.
        # Flag loudly instead of silently returning the "closest" rotation.
        id_flag = bool(det_fire and sh[0] > 0.0
                       and sh[2] / sh[0] > _DEPTH_DET_FIRE_S3_RATIO)

        resid = (a @ r_ct.T + t_ct) - b
        sq = np.sum(resid * resid, axis=1)
        t_ct_mat = np.eye(4)
        t_ct_mat[:3, :3] = r_ct
        t_ct_mat[:3, 3] = t_ct
        poses.append({
            'index': idx,
            't_bg': np.asarray(s['T_bg'], float).reshape(4, 4),
            't_ct': t_ct_mat,
            'rms_mm': 1000.0 * math.sqrt(float(np.mean(sq))),
            'sq_sum': float(np.sum(sq)),
            'n_pts': len(gids),
            'model_min_sval': float(sv_model[-1]),
            'id_flag': id_flag,
        })

    per_pose_rms_mm = [p['rms_mm'] for p in poses]
    n_pts_total = sum(p['n_pts'] for p in poses)
    rms_mm = (1000.0 * math.sqrt(sum(p['sq_sum'] for p in poses) / n_pts_total)
              if n_pts_total else float('nan'))
    base_out = {
        'ok': False, 'X': None, 'T_bt': None,
        'per_pose_rms_mm': per_pose_rms_mm, 'rms_mm': rms_mm,
        'n_poses': len(poses),
        'T_ct_list': [p['t_ct'] for p in poses],
        'pose_indices': [p['index'] for p in poses],
        'per_pose_model_min_sval': [p['model_min_sval'] for p in poses],
        'id_association_flags': [p['index'] for p in poses if p['id_flag']],
        'skipped': skipped,
        'R_x_source': None,
        'axis_singular_values': None, 'n_axis_pairs': 0,
        'translation_cond': None,
        'consistency_rms_mm': None, 'consistency_rot_rms_deg': None,
    }
    if len(poses) < 2:
        base_out['msg'] = ('need >=2 registrable poses to separate X from T_bt '
                           '(got %d usable, %d skipped)' % (len(poses), len(skipped)))
        return base_out

    # ---- step 2: R_x by axis-Wahba on AX=XB pair axes ----
    # From T_bt = T_bg(i) X T_ct(i) = T_bg(j) X T_ct(j):
    #   A_ij X = X B_ij with A_ij = R_bg(j)^T R_bg(i), B_ij = R_ct(j) R_ct(i)^T.
    # Conjugation A = R_x B R_x^T maps rotation vectors: axis(A) = R_x axis(B),
    # with matched signs (Rodrigues returns the canonical theta in [0,pi]).
    # Weight each pair by min(theta_A, theta_B): small rotations have noisy axes.
    m_axes = np.zeros((3, 3))
    n_pairs = 0
    min_axis_rad = math.radians(_DEPTH_AXIS_MIN_ANGLE_DEG)
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            a_rel = poses[j]['t_bg'][:3, :3].T @ poses[i]['t_bg'][:3, :3]
            b_rel = poses[j]['t_ct'][:3, :3] @ poses[i]['t_ct'][:3, :3].T
            ra = cv2.Rodrigues(np.ascontiguousarray(a_rel))[0].reshape(3)
            rb = cv2.Rodrigues(np.ascontiguousarray(b_rel))[0].reshape(3)
            tha, thb = float(np.linalg.norm(ra)), float(np.linalg.norm(rb))
            if min(tha, thb) < min_axis_rad:
                continue
            m_axes += min(tha, thb) * np.outer(ra / tha, rb / thb)
            n_pairs += 1
    if n_pairs == 0:
        base_out['msg'] = ('no pose pair has relative rotation >= %.1f deg -- '
                           'R_x unobservable from depth alone; supply X0 externally'
                           % _DEPTH_AXIS_MIN_ANGLE_DEG)
        return base_out
    r_x = _depth_proper_rot(m_axes)
    # s2/s1, s3/s1 of the axis bundle ARE the AX=XB conditioning: with J1+J4-only
    # motion expect two strong directions and a weak third -- report, don't hide.
    axis_svals = np.linalg.svd(m_axes, compute_uv=False)

    # ---- step 3: R_bt chordal mean + joint linear (t_x, t_bt) ----
    # Each pose predicts R_bt(i) = R_bg(i) R_x R_ct(i); average on SO(3) via the
    # chordal (SVD-projected) mean rather than trusting any single pose.
    r_bt_stack = [p['t_bg'][:3, :3] @ r_x @ p['t_ct'][:3, :3] for p in poses]
    r_bt = _depth_proper_rot(np.sum(r_bt_stack, axis=0))

    # Translation identity per pose:  t_bt - R_bg(i) t_x = R_bg(i) R_x t_ct(i) + t_bg(i)
    # Stacked as [ -R_bg(i) | I3 ] [t_x; t_bt] = rhs -> one 3Nx6 LSQ over ALL poses.
    # Null vectors would need (R_bg(i)-R_bg(j)) t_x = 0 for all pairs; with both
    # J1 and J4 exercised only t_x = 0 qualifies -> (t_x, t_bt) fully observable.
    # The condition number is reported for the J1-only degenerate case.
    a_stack = np.zeros((3 * len(poses), 6))
    rhs = np.zeros(3 * len(poses))
    for k, p in enumerate(poses):
        r_bg = p['t_bg'][:3, :3]
        a_stack[3 * k:3 * k + 3, 0:3] = -r_bg
        a_stack[3 * k:3 * k + 3, 3:6] = np.eye(3)
        rhs[3 * k:3 * k + 3] = r_bg @ r_x @ p['t_ct'][:3, 3] + p['t_bg'][:3, 3]
    sol, _res, _rank, sv_stack = np.linalg.lstsq(a_stack, rhs, rcond=None)
    t_x, t_bt = sol[0:3], sol[3:6]
    trans_cond = (float(sv_stack[0] / sv_stack[-1])
                  if sv_stack[-1] > 0 else float('inf'))

    x_mat = np.eye(4)
    x_mat[:3, :3] = r_x
    x_mat[:3, 3] = t_x
    t_bt_mat = np.eye(4)
    t_bt_mat[:3, :3] = r_bt
    t_bt_mat[:3, 3] = t_bt

    # Consistency spread of T_bg(i) X T_ct(i) about T_bt: this is the number that
    # exposes a bad R_x even when every per-pose Kabsch residual is perfect
    # (per-pose registration quality and cross-pose chain consistency are
    # independent failure modes).
    trans_resid = (a_stack @ sol - rhs).reshape(-1, 3)
    consistency_rms_mm = 1000.0 * math.sqrt(float(np.mean(np.sum(trans_resid ** 2, axis=1))))
    rot_devs = [_depth_rot_angle_deg(r_bt.T @ ri) for ri in r_bt_stack]
    consistency_rot_rms_deg = math.sqrt(float(np.mean(np.square(rot_devs))))

    base_out.update({
        'ok': True,
        'X': x_mat,
        'T_bt': t_bt_mat,
        'R_x_source': 'axb_axes',
        'axis_singular_values': [float(v) for v in axis_svals],
        'n_axis_pairs': n_pairs,
        'translation_cond': trans_cond,
        'consistency_rms_mm': consistency_rms_mm,
        'consistency_rot_rms_deg': consistency_rot_rms_deg,
        'msg': ('depth anchor: %d poses (%d skipped), %d corners, '
                'registration rms %.3f mm, chain consistency %.3f mm / %.3f deg'
                % (len(poses), len(skipped), n_pts_total, rms_mm,
                   consistency_rms_mm, consistency_rot_rms_deg)),
    })
    if base_out['id_association_flags']:
        base_out['msg'] += ('; WARNING: reflection detected on well-conditioned '
                            'pose(s) %s -- check corner id association'
                            % base_out['id_association_flags'])
    return base_out

# bias component along the arc-invariant axis is inseparable from a yaw tweak
# of R_x (see solve_gravity's conditioning report); this prior bounds that
# near-null direction instead of letting it wander.
_FUS_BIAS_PRIOR_MPS2 = 0.05

# Collinearity guard for the per-pose init Kabsch: second singular value of the
# centred model points must exceed this fraction of the first (a collinear set
# leaves a 1-DOF rotation family — useless as an init, skip the pose).
_FUS_COLLINEAR_TOL = 1e-3


def _fus_rvec_to_R(r):
    return cv2.Rodrigues(np.asarray(r, float).reshape(3, 1))[0]


def _fus_R_to_rvec(R):
    # ascontiguousarray: cv2.Rodrigues rejects the non-contiguous views numpy
    # produces for T[:3, :3] slices.
    return cv2.Rodrigues(np.ascontiguousarray(R, dtype=float))[0].reshape(3)


def _fus_project_SO3(M):
    """Nearest rotation (chordal/Frobenius) to M with det=+1 enforced.

    The diag(1,1,sign(det)) correction is load-bearing here, not hygiene:
    rank-deficient inputs (coplanar point sets, single-arc rotation ensembles)
    can otherwise come back as reflections."""
    U, _s, Vt = np.linalg.svd(np.asarray(M, float))
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(U @ Vt)))])
    return U @ D @ Vt


def _fus_kabsch_ct(a_pts, b_pts):
    """3D-3D registration  b ≈ R·a + t  (a: target frame, b: camera frame, m).

    Returns (T 4x4, rms_m) or (None, 'collinear') when the model points span
    a line.  Coplanar is fine: rank-2 H with the det-correction still yields
    the unique proper rotation (no IPPE-style twofold ambiguity in 3D-3D)."""
    a = np.asarray(a_pts, float).reshape(-1, 3)
    b = np.asarray(b_pts, float).reshape(-1, 3)
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    A = a - ac
    s = np.linalg.svd(A, compute_uv=False)
    if s[0] <= 0.0 or s[1] / s[0] < _FUS_COLLINEAR_TOL:
        return None, 'collinear'
    H = A.T @ (b - bc)                       # Σ (a-ā)(b-b̄)^T
    U, _s2, Vt = np.linalg.svd(H)
    V = Vt.T
    D = np.diag([1.0, 1.0, float(np.sign(np.linalg.det(V @ U.T)))])
    R = V @ D @ U.T
    t = bc - R @ ac
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    rms = float(np.sqrt(np.mean(np.sum((a @ R.T + t - b) ** 2, axis=1))))
    return T, rms


def _fus_tangent_basis(f):
    """Orthonormal (e1, e2) spanning the plane perpendicular to unit f.

    Anchored at the MEASURED direction, hence state-independent — a basis that
    moved with the state would make the finite-difference Jacobian noisy."""
    f = np.asarray(f, float).reshape(3)
    a = np.zeros(3)
    a[int(np.argmin(np.abs(f)))] = 1.0        # axis least aligned with f
    e1 = np.cross(f, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(f, e1)
    return e1, e2


def _fus_sigma_active(sigma):
    """sigma=None or non-finite/non-positive disables that residual block
    (documented ablation hook — the public signature stays fixed)."""
    return sigma is not None and np.isfinite(sigma) and sigma > 0.0


def solve_fusion(samples, model_pts_mm, K, dist, X0=None, T_bt0=None,
                 sigma_px=0.7, sigma_depth_m=0.004, sigma_grav_rad=0.005,
                 estimate_bias=False, robust=True):
    """Weighted nonlinear least squares over [r_x, t_x, r_bt, t_bt, (b_a)].

    FK is a HARD constraint (T_bg not optimised).  Residual blocks, each
    Mahalanobis-whitened to dimensionless:
      vision : (1/sigma_px)       * (uv_obs - project(K, dist, X^-1·T_bg^-1·T_bt·P_j))
      depth  : (1/sigma_depth_m)  * (q_cam  -  X^-1·T_bg^-1·T_bt·P_j)
      gravity: (1/sigma_grav_rad) * tangent-plane components of (R_x^T u_i - f_i)
               — 2 DOF per pose: both vectors are unit so the radial component
               is O(theta^2); including it would add a fictitious stiff
               direction and corrupt the whitening.
      bias prior (estimate_bias only): b_a / 0.05 m/s^2 — regularises the
               J4-arc-unobservable bias axis.

    samples : list of per-pose dicts; every anchor key optional per pose:
        {'T_bg':     (4,4) base<-gripper, REQUIRED (FK, metres),
         'pts_cam_m':{gid: (3,)}  depth points in camera COLOR frame (m),
         'uv_px':    {gid: (2,)}  observed pixel of corner gid,
         'f':        (3,)         unit specific force (+up) in COLOR frame,
         'a_raw':    (3,)         m/s^2 unnormalised mean accel (COLOR frame;
                                  needed only when estimate_bias=True)}
      A pose missing a key contributes no residuals of that type.  u_i is
      derived internally as R_bg^T·[0,0,1] (single source of truth).
    model_pts_mm : {gid: (3,) target-frame mm} — converted to metres once here.
    Passing sigma_*=None (or inf) drops that block entirely (ablation runs).

    Initialisation when X0/T_bt0 are absent: R_x from solve_gravity (fallback
    solve_depth's AX=XB axes), per-pose T_ct from depth Kabsch, R_bt chordal
    mean, (t_x, t_bt) from the linear 3Nx6 stack.  No PnP anywhere.  If depth
    data is absent, X0 and T_bt0 are mandatory (raises ValueError).

    Returns {'X' (4,4), 'T_bt' (4,4), 'bias' (3,) or None,
             'rms_px', 'rms_depth_mm', 'rms_grav_deg'  (physical units,
             None for disabled/empty blocks), 'success', 'n_resid',
             'jacobian_cond', 'n_vision_poses', 'n_depth_poses',
             'n_gravity_poses', 'n_vision_corners', 'n_depth_points',
             'cost', 'msg'}.
    """
    from scipy.optimize import least_squares

    K = np.asarray(K, float).reshape(3, 3)
    dvec = np.zeros(5)                        # dist padded to 5 for OpenCV
    if dist is not None:
        dd = np.asarray(dist, float).reshape(-1)
        dvec[:min(5, dd.size)] = dd[:5]

    # mm -> m exactly once, at ingest (SPEC convention).
    model_m = {int(g): np.asarray(p, float).reshape(3) / 1000.0
               for g, p in model_pts_mm.items()}

    use_vis = _fus_sigma_active(sigma_px)
    use_dep = _fus_sigma_active(sigma_depth_m)
    use_grav = _fus_sigma_active(sigma_grav_rad)

    # ---- ingest poses (all anchor data parsed regardless of block flags:
    #      disabled blocks contribute no residuals but their data may still
    #      seed the initialisation chain) ----
    poses = []
    for s in samples:
        T_bg = np.asarray(s['T_bg'], float).reshape(4, 4)
        p = {'R_bg': T_bg[:3, :3].copy(), 't_bg': T_bg[:3, 3].copy()}
        # 'uv_px' 与 'uv' 都接受（两个调用方各自沿用的键名）。
        # 注意：这里若取不到，视觉块会被**静默**跳过——曾导致融合只用深度+重力
        # 而平移错 123mm，且外部看不出来。故下方对 0 视觉位姿显式告警。
        uv = s.get('uv_px', s.get('uv'))
        if uv:
            gids = [g for g in sorted(int(k) for k in uv) if g in model_m]
            if gids:
                p['vis_obj'] = np.ascontiguousarray(
                    [model_m[g] for g in gids], dtype=float)
                p['vis_img'] = np.asarray(
                    [uv[g] for g in gids], float).reshape(-1, 2)
        pc = s.get('pts_cam_m')
        if pc:
            gids = [g for g in sorted(int(k) for k in pc) if g in model_m]
            if gids:
                p['dep_obj'] = np.asarray([model_m[g] for g in gids])
                p['dep_cam'] = np.asarray(
                    [pc[g] for g in gids], float).reshape(-1, 3)
        f_raw = s.get('f')
        a_raw = s.get('a_raw')
        fvec = None
        if f_raw is not None:
            fvec = np.asarray(f_raw, float).reshape(3)
        elif a_raw is not None:
            fvec = np.asarray(a_raw, float).reshape(3)
        if fvec is not None:
            nf = float(np.linalg.norm(fvec))
            if nf > 1e-6:
                f_unit = fvec / nf
                # u_i = R_bg^T [0,0,1] == third ROW of R_bg
                u = p['R_bg'][2, :].copy()
                u /= np.linalg.norm(u)
                e1, e2 = _fus_tangent_basis(f_unit)
                p['f'], p['u'], p['e1'], p['e2'] = f_unit, u, e1, e2
                if a_raw is not None:
                    p['a_raw'] = np.asarray(a_raw, float).reshape(3)
        poses.append(p)

    vis_poses = [p for p in poses if use_vis and 'vis_obj' in p]
    dep_poses = [p for p in poses if use_dep and 'dep_obj' in p]
    grav_poses = [p for p in poses if use_grav and 'f' in p]

    n_vis_corners = int(sum(len(p['vis_obj']) for p in vis_poses))
    n_dep_points = int(sum(len(p['dep_obj']) for p in dep_poses))

    # Bias only makes sense with magnitudes: unit vectors carry no bias
    # information (SPEC inconsistency #1) -> require a_raw on gravity poses.
    bias_poses = [p for p in grav_poses if 'a_raw' in p]
    opt_bias = bool(estimate_bias and use_grav and bias_poses)
    bias_msg = ''
    if estimate_bias and not opt_bias:
        bias_msg = ('; estimate_bias requested but no usable a_raw '
                    '(or gravity block disabled) -> bias not estimated')

    n_params = 15 if opt_bias else 12

    def _fail(msg):
        return {'X': None, 'T_bt': None, 'bias': None,
                'rms_px': None, 'rms_depth_mm': None, 'rms_grav_deg': None,
                'success': False, 'n_resid': 0, 'jacobian_cond': float('nan'),
                'n_vision_poses': len(vis_poses),
                'n_depth_poses': len(dep_poses),
                'n_gravity_poses': len(grav_poses),
                'n_vision_corners': n_vis_corners,
                'n_depth_points': n_dep_points,
                'cost': float('nan'), 'msg': msg + bias_msg}

    # ---- initialisation chain (SPEC §5) ----
    X0m = None if X0 is None else np.asarray(X0, float).reshape(4, 4)
    T_bt0m = None if T_bt0 is None else np.asarray(T_bt0, float).reshape(4, 4)

    if X0m is None or T_bt0m is None:
        # Per-pose T_ct from depth Kabsch on well-conditioned poses
        # (data used even when the depth residual block is sigma-disabled).
        init_ct = []
        for p in poses:
            if 'dep_obj' in p and len(p['dep_obj']) >= 3:
                T_ct, _info = _fus_kabsch_ct(p['dep_obj'], p['dep_cam'])
                if T_ct is not None:
                    init_ct.append((p, T_ct))

        # R_x seed: gravity Wahba preferred; depth AX=XB axes as fallback.
        # No blind multi-start — the anchors exist precisely to avoid it.
        R_x0 = None if X0m is None else X0m[:3, :3].copy()
        grav_pairs = [{'u': p['u'], 'f': p['f']} for p in poses if 'f' in p]
        if R_x0 is None and len(grav_pairs) >= 2:
            R_x0 = np.asarray(solve_gravity(grav_pairs)['R_x'], float)
        if R_x0 is None and len(init_ct) >= 2:
            try:
                dep_samples = [{'T_bg': s['T_bg'], 'pts_cam_m': s['pts_cam_m']}
                               for s in samples if s.get('pts_cam_m')]
                res_d = solve_depth(dep_samples, model_pts_mm)
                R_x0 = np.asarray(res_d['X'], float)[:3, :3]
            except Exception:
                R_x0 = None
        if R_x0 is None:
            raise ValueError(
                'solve_fusion: cannot initialise R_x — supply X0 or provide '
                'gravity (>=2 poses with f) or depth (>=2 usable poses) data')

        if X0m is not None:
            # T_bt from the chain identity averaged over depth poses.
            if not init_ct:
                raise ValueError(
                    'solve_fusion: T_bt0 missing and no usable depth poses to '
                    'derive it — supply T_bt0')
            Tl = [np.eye(4) for _ in init_ct]
            for k, (p, T_ct) in enumerate(init_ct):
                T_bg = np.eye(4)
                T_bg[:3, :3], T_bg[:3, 3] = p['R_bg'], p['t_bg']
                Tl[k] = T_bg @ X0m @ T_ct
            R_bt0 = _fus_project_SO3(np.sum([T[:3, :3] for T in Tl], axis=0))
            t_bt0 = np.mean([T[:3, 3] for T in Tl], axis=0)
        elif T_bt0m is not None:
            # t_x from  t_x = R_bg^T (t_bt - t_bg) - R_x·t_ct  averaged.
            if not init_ct:
                raise ValueError(
                    'solve_fusion: X0 missing and no usable depth poses to '
                    'derive t_x — supply X0')
            t_x0 = np.mean(
                [p['R_bg'].T @ (T_bt0m[:3, 3] - p['t_bg']) - R_x0 @ T_ct[:3, 3]
                 for p, T_ct in init_ct], axis=0)
            R_bt0, t_bt0 = T_bt0m[:3, :3].copy(), T_bt0m[:3, 3].copy()
            X0m = np.eye(4)
            X0m[:3, :3], X0m[:3, 3] = R_x0, t_x0
        else:
            if len(init_ct) < 2:
                raise ValueError(
                    'solve_fusion: depth data absent/insufficient (<2 usable '
                    'poses) — X0 and T_bt0 are mandatory inputs in that case')
            R_bt0 = _fus_project_SO3(
                np.sum([p['R_bg'] @ R_x0 @ T_ct[:3, :3]
                        for p, T_ct in init_ct], axis=0))
            # Linear 3Nx6 stack:  t_bt - R_bg·t_x = R_bg·R_x·t_ct + t_bg
            A_rows, y_rows = [], []
            for p, T_ct in init_ct:
                A_rows.append(np.hstack([-p['R_bg'], np.eye(3)]))
                y_rows.append(p['R_bg'] @ (R_x0 @ T_ct[:3, 3]) + p['t_bg'])
            sol6 = np.linalg.lstsq(np.vstack(A_rows),
                                   np.concatenate(y_rows), rcond=None)[0]
            t_x0, t_bt0 = sol6[:3], sol6[3:]
            X0m = np.eye(4)
            X0m[:3, :3], X0m[:3, 3] = R_x0, t_x0
        if T_bt0m is None:
            T_bt0m = np.eye(4)
            T_bt0m[:3, :3], T_bt0m[:3, 3] = R_bt0, t_bt0

    # cv2.Rodrigues returns the canonical vector (||r|| <= pi); the optimiser
    # stays local so no in-loop wrapping is needed.
    theta0 = np.concatenate([
        _fus_R_to_rvec(X0m[:3, :3]), X0m[:3, 3],
        _fus_R_to_rvec(T_bt0m[:3, :3]), T_bt0m[:3, 3],
        np.zeros(3) if opt_bias else np.zeros(0)])

    # ---- residual assembly ----
    def _eval(theta):
        """Unwhitened per-block residuals (shared by optimiser + reporting so
        the two can never diverge).  Returns dict of raveled arrays:
        'vis' px, 'dep' m, 'grav_tan' rad (2/pose), 'grav_ang_deg' (1/pose)."""
        R_x = _fus_rvec_to_R(theta[0:3])
        t_x = theta[3:6]
        R_bt = _fus_rvec_to_R(theta[6:9])
        t_bt = theta[9:12]
        b = theta[12:15] if opt_bias else None
        RxT = R_x.T

        def ct_of(p):
            # T_ct = X^-1 · T_bg^-1 · T_bt  with analytic rigid inverses
            R_ct = RxT @ p['R_bg'].T @ R_bt
            t_ct = RxT @ (p['R_bg'].T @ (t_bt - p['t_bg']) - t_x)
            return R_ct, t_ct

        out = {'vis': [], 'dep': [], 'grav_tan': [], 'grav_ang_deg': []}
        for p in vis_poses:
            R_ct, t_ct = ct_of(p)
            rvec = _fus_R_to_rvec(R_ct)
            proj, _ = cv2.projectPoints(p['vis_obj'], rvec.reshape(3, 1),
                                        t_ct.reshape(3, 1), K, dvec)
            out['vis'].append((p['vis_img'] - proj.reshape(-1, 2)).ravel())
        for p in dep_poses:
            R_ct, t_ct = ct_of(p)
            q_pred = p['dep_obj'] @ R_ct.T + t_ct
            out['dep'].append((p['dep_cam'] - q_pred).ravel())
        for p in grav_poses:
            f = p['f']
            if opt_bias and 'a_raw' in p:
                af = p['a_raw'] - b
                na = float(np.linalg.norm(af))
                if na > 1e-9:
                    f = af / na
            v = RxT @ p['u']
            e = v - f
            # 2-DOF tangent projection at the measured direction: exact to
            # 1st order the tangential components equal the angular error (rad)
            out['grav_tan'].append(np.array([p['e1'] @ e, p['e2'] @ e]))
            out['grav_ang_deg'].append(np.degrees(
                np.arccos(np.clip(float(np.dot(v, f)), -1.0, 1.0))))
        for key in ('vis', 'dep', 'grav_tan'):
            out[key] = (np.concatenate(out[key]) if out[key]
                        else np.zeros(0))
        return out

    def residuals(theta):
        ev = _eval(theta)
        parts = [ev['vis'] / sigma_px if use_vis else np.zeros(0),
                 ev['dep'] / sigma_depth_m if use_dep else np.zeros(0),
                 ev['grav_tan'] / sigma_grav_rad if use_grav else np.zeros(0)]
        if opt_bias:
            parts.append(theta[12:15] / _FUS_BIAS_PRIOR_MPS2)
        return np.concatenate(parts)

    r0 = residuals(theta0)
    n_resid = int(r0.size)
    if n_resid <= n_params:
        return _fail('need > %d residuals, got %d (all blocks empty or '
                     'disabled?)' % (n_params, n_resid))
    if not np.all(np.isfinite(r0)):
        return _fail('non-finite residuals at the initial point')

    try:
        sol = least_squares(
            residuals, theta0, method='trf',
            loss='soft_l1' if robust else 'linear',
            f_scale=1.0,          # whitened residuals: robust knee at 1 sigma
            x_scale='jac', max_nfev=400)
    except Exception as exc:
        return _fail('least_squares failed: %s' % exc)

    th = sol.x
    X = np.eye(4)
    X[:3, :3] = _fus_rvec_to_R(th[0:3])
    X[:3, 3] = th[3:6]
    T_bt = np.eye(4)
    T_bt[:3, :3] = _fus_rvec_to_R(th[6:9])
    T_bt[:3, 3] = th[9:12]
    bias_out = th[12:15].copy() if opt_bias else None

    # ---- per-block RMS in PHYSICAL units (unwhitened) ----
    ev = _eval(th)
    rms_px = (float(np.sqrt(np.mean(ev['vis'] ** 2)))
              if ev['vis'].size else None)
    rms_depth_mm = (1000.0 * float(np.sqrt(np.mean(ev['dep'] ** 2)))
                    if ev['dep'].size else None)
    ga = np.asarray(ev['grav_ang_deg'], float)
    rms_grav_deg = float(np.sqrt(np.mean(ga ** 2))) if ga.size else None

    # Jacobian condition number (observability diagnostic, as in handeye_ba)
    try:
        jac = sol.jac if getattr(sol, 'jac', None) is not None else None
        cond = float(np.linalg.cond(jac)) if jac is not None else float('nan')
    except Exception:
        cond = float('nan')

    success = bool(sol.success) and bool(np.all(np.isfinite(th)))
    msg = ('anchor fusion: %d poses (vis %d/%d corners, depth %d/%d pts, '
           'grav %d) rms [%s px, %s mm, %s deg]%s' % (
               len(poses), len(vis_poses), n_vis_corners,
               len(dep_poses), n_dep_points, len(grav_poses),
               'n/a' if rms_px is None else '%.3f' % rms_px,
               'n/a' if rms_depth_mm is None else '%.2f' % rms_depth_mm,
               'n/a' if rms_grav_deg is None else '%.3f' % rms_grav_deg,
               bias_msg))

    # 某一路锚点全程 0 观测，几乎总是键名/装配错误而非有意消融。静默跳过会
    # 伪装成"求解器精度差"(实测：视觉块被跳过 → 平移错 123mm 却无任何提示)。
    for _blk, _n, _sig in (('vision', len(vis_poses), sigma_px),
                           ('depth', len(dep_poses), sigma_depth_m),
                           ('gravity', len(grav_poses), sigma_grav_rad)):
        if _n == 0 and _fus_sigma_active(_sig):
            msg += ('; WARNING: %s block enabled but got 0 poses — check sample '
                    'key names' % _blk)

    # 【实测教训 2026-07-19】鲁棒损失会牺牲重力块。
    # 重力只有 2×n_pose 个残差(本例 36)，深度/视觉各上千。soft_l1 把大残差代价
    # 封顶后，"放弃这 36 个高精度约束"变得很便宜：真实数据上 robust=True 使
    # 重力残差从 3.9° 恶化到 24.7°，解偏离重力锚 25°。
    # 鲁棒损失是给"多观测、有外点"的块用的，不该套在少而精的物理锚上。
    # 正解是分块损失(锚点用二次、角点观测用 soft_l1)；在此之前，含重力块时
    # 请用 robust=False。
    if robust and len(grav_poses) and n_dep_points + n_vis_corners > 10 * len(grav_poses):
        msg += ('; WARNING: robust=True with a small gravity block '
                '(%d gravity residual pairs vs %d corner observations) — the '
                'robust kernel can abandon the gravity anchor; verify against '
                'solve_gravity and prefer robust=False'
                % (len(grav_poses), n_dep_points + n_vis_corners))

    return {'X': X, 'T_bt': T_bt, 'bias': bias_out,
            'rms_px': rms_px, 'rms_depth_mm': rms_depth_mm,
            'rms_grav_deg': rms_grav_deg,
            'success': success, 'n_resid': n_resid,
            'jacobian_cond': cond,
            'n_vision_poses': len(vis_poses),
            'n_depth_poses': len(dep_poses),
            'n_gravity_poses': len(grav_poses),
            'n_vision_corners': n_vis_corners,
            'n_depth_points': n_dep_points,
            'cost': float(sol.cost), 'msg': msg}
