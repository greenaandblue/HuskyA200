"""2D occupied / free grids from scans (ray casting), and a free-space-aware global search."""
import numpy as np
from scipy import ndimage
from scipy.signal import fftconvolve
from register import se2, ZWALL


def scan_rays(d, T=np.eye(4), every=15, nray=1500, z=ZWALL, seed=0):
    """Return list of (origin_xy, hits_xy) in frame T·map, for every `every`-th scan."""
    rng = np.random.default_rng(seed)
    out = []
    P, idx, pts = d['poses'], d['idx'], d['pts']
    for i in range(0, len(P), every):
        p = pts[idx[i]:idx[i + 1], :3].astype(np.float64)
        p = p[(p[:, 2] > z[0]) & (p[:, 2] < z[1])]
        if len(p) > nray:
            p = p[rng.choice(len(p), nray, replace=False)]
        M = T @ P[i]
        w = p @ M[:3, :3].T + M[:3, 3]
        out.append((M[:2, 3], w[:, :2]))
    return out


def grids(rays, res, lo, shape):
    occ = np.zeros(shape, np.int32); free = np.zeros(shape, np.int32)
    for o, h in rays:
        v = h - o; L = np.linalg.norm(v, axis=1)
        n = int(np.ceil(L.max() / (res * 0.5)))
        s = np.linspace(0, 1, n, endpoint=False)[:, None, None]       # (n,1,1)
        q = o + s * v[None]                                            # (n,R,2)
        keep = (s[..., 0] * L[None] < L[None] - 2 * res)               # stop short of the hit
        q = q[keep]
        ij = np.floor((q - lo) / res).astype(int)
        ok = (ij >= 0).all(1) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
        np.add.at(free, (ij[ok, 0], ij[ok, 1]), 1)
        ij = np.floor((h - lo) / res).astype(int)
        ok = (ij >= 0).all(1) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
        np.add.at(occ, (ij[ok, 0], ij[ok, 1]), 1)
    # static occupancy: hit repeatedly and not mostly seen through (people walking with the robot)
    ratio = occ / np.maximum(occ + free, 1)
    O = (occ >= 2) & (ratio > 0.25)
    F = (free >= 3) & (ratio < 0.05) & ~ndimage.binary_dilation(O, iterations=2)
    return O.astype(np.float32), F.astype(np.float32)


def search(src_rays, dst_rays, res=0.15, yaw_step=1.0, lam=2.0, top=10, blur=1.0):
    """Exhaustive (yaw, x, y) search maximizing occ overlap - lam * (occ-in-free conflicts, both ways)."""
    allp = np.vstack([h for _, h in dst_rays] + [o[None] for o, _ in dst_rays])
    lo = allp.min(0) - 25; hi = allp.max(0) + 25
    shape = tuple(np.ceil((hi - lo) / res).astype(int))
    Do, Df = grids(dst_rays, res, lo, shape)
    Dob = ndimage.gaussian_filter(Do, blur); Dob /= Dob.max()
    # src grids computed once, then their cell centres are rotated per yaw (cheap)
    sp = np.vstack([h for _, h in src_rays] + [o[None] for o, _ in src_rays])
    slo0 = sp.min(0) - 25; sshape0 = tuple(np.ceil((sp.max(0) + 25 - slo0) / res).astype(int))
    So0, Sf0 = grids(src_rays, res, slo0, sshape0)
    occ_c = (np.argwhere(So0 > 0) + 0.5) * res + slo0
    free_c = (np.argwhere(Sf0 > 0) + 0.5) * res + slo0
    c0 = occ_c.mean(0)
    f = lambda A, B: fftconvolve(A, B[::-1, ::-1], mode='full')
    cands = []
    for yd in np.arange(0, 360, yaw_step):
        y = np.deg2rad(yd); Rm = np.array([[np.cos(y), -np.sin(y)], [np.sin(y), np.cos(y)]])
        ro = (occ_c - c0) @ Rm.T; rf = (free_c - c0) @ Rm.T
        slo = np.minimum(ro.min(0), rf.min(0)) - res
        sshape = tuple(np.ceil((np.maximum(ro.max(0), rf.max(0)) - slo) / res).astype(int) + 2)
        So = np.zeros(sshape, np.float32); Sf = np.zeros(sshape, np.float32)
        So[tuple(np.floor((ro - slo) / res).astype(int).T)] = 1
        Sf[tuple(np.floor((rf - slo) / res).astype(int).T)] = 1
        n = So.sum()
        score = (f(Dob, So) - lam * f(Df, So) - lam * f(Do, Sf)) / n
        flat = np.argpartition(score.ravel(), -3)[-3:]
        for fl in flat:
            i, j = np.unravel_index(fl, score.shape)
            off = np.array([i - (sshape[0] - 1), j - (sshape[1] - 1)]) * res + lo
            T = np.eye(4); T[:2, :2] = Rm; T[:2, 3] = off - slo - Rm @ c0
            cands.append((float(score[i, j]), yd, T))
    cands.sort(key=lambda x: -x[0])
    keep = []
    for sc, yd, T in cands:
        if all(np.linalg.norm(T[:2, 3] - K[:2, 3]) > 1.5 or abs(((yd - ky + 180) % 360) - 180) > 6 for _, ky, K in keep):
            keep.append((sc, yd, T))
        if len(keep) >= top:
            break
    return keep, (Do, Df, lo, res)
