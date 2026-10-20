"""Registration helpers between room maps (all planar: x, y, yaw)."""
import numpy as np, open3d as o3d
from scipy import ndimage
from scipy.signal import fftconvolve
from viz import LIDAR_IN_BASE

ZWALL = (-0.6, 1.2)   # slice relative to lidar height (floor ~ -1.03): walls / exhibits, no floor/ceiling


def se2(x, y, yaw):
    T = np.eye(4); c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]; T[:2, 3] = [x, y]; return T


def planar(T):
    return se2(T[0, 3], T[1, 3], np.arctan2(T[1, 0], T[0, 0]))


def walls(pc, z=ZWALL, voxel=0.1):
    a = np.asarray(pc.points); m = (a[:, 2] > z[0]) & (a[:, 2] < z[1])
    w = pc.select_by_index(np.where(m)[0]).voxel_down_sample(voxel)
    w.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
    return w


def icp_planar(src, dst, init, dists=(1.0, 0.5, 0.25, 0.12)):
    """Coarse-to-fine point-to-plane ICP, result projected to SE(2) each stage."""
    T = init
    for d in dists:
        r = o3d.pipelines.registration.registration_icp(
            src, dst, d, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(
                o3d.pipelines.registration.TukeyLoss(k=d)),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = planar(r.transformation)
    ev = o3d.pipelines.registration.evaluate_registration(src, dst, 0.15, T)
    return T, ev.fitness, ev.inlier_rmse


def grid(xy, res, origin, shape):
    ij = np.floor((xy - origin) / res).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    g = np.zeros(shape, np.float32); np.add.at(g, (ij[ok, 0], ij[ok, 1]), 1)
    return (g > 0).astype(np.float32)


def global_2d(src_xy, dst_xy, res=0.1, yaw_step_deg=1.0, top=10, blur=1.5):
    """Exhaustive correlative matching: for each yaw, FFT cross-correlate occupancy grids.
    Returns list of (score, T src->dst) sorted best first (non-max suppressed)."""
    lo = dst_xy.min(0) - 2; hi = dst_xy.max(0) + 2
    shape = tuple(np.ceil((hi - lo) / res).astype(int))
    D = ndimage.gaussian_filter(grid(dst_xy, res, lo, shape), blur)
    c0 = src_xy.mean(0)
    cands = []
    for yd in np.arange(0, 360, yaw_step_deg):
        y = np.deg2rad(yd); c, s = np.cos(y), np.sin(y)
        p = (src_xy - c0) @ np.array([[c, -s], [s, c]]).T
        slo = p.min(0); sshape = tuple(np.ceil((p.max(0) - slo) / res).astype(int) + 1)
        S = grid(p, res, slo, sshape)
        n = S.sum()
        C = fftconvolve(D, S[::-1, ::-1], mode='full') / n
        # top peaks for this yaw
        flat = np.argsort(C.ravel())[-5:]
        for f in flat:
            i, j = np.unravel_index(f, C.shape)
            # offset of S's origin in D grid
            off = np.array([i - (sshape[0] - 1), j - (sshape[1] - 1)]) * res + lo
            t = off - slo  # translation applied after rotation (on centred points)
            T = se2(0, 0, 0); T[:2, :2] = [[c, -s], [s, c]]
            T[:2, 3] = t - T[:2, :2] @ c0
            cands.append((float(C[i, j]), yd, T))
    cands.sort(key=lambda x: -x[0])
    keep = []
    for sc, yd, T in cands:
        if all(np.linalg.norm(T[:2, 3] - K[:2, 3]) > 1.0 or abs(((yd - ky + 180) % 360) - 180) > 5 for _, ky, K in keep):
            keep.append((sc, yd, T))
        if len(keep) >= top:
            break
    return keep


def odom_link(dA, dB):
    """Transform mapB -> mapA from continuous wheel odom (same boot)."""
    LA = dA['odom'][0] @ LIDAR_IN_BASE
    LB = dB['odom'][0] @ LIDAR_IN_BASE
    return planar(np.linalg.inv(LA) @ LB)
