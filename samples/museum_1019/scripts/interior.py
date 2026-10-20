import numpy as np
from scipy import ndimage
A = np.load('/home/claude/work/merged_xyzr.npy')
RES = 0.1


def interior_mask(r_close, min_pts=3):
    x, y, z = A[:, 0], A[:, 1], A[:, 2]
    lo = np.array([x.min(), y.min()]) - 3
    shape = tuple(np.ceil((np.array([x.max(), y.max()]) + 3 - lo) / RES).astype(int))
    m = (z > 0.3) & (z < 2.5)
    ij = np.floor((np.stack([x[m], y[m]], 1) - lo) / RES).astype(int)
    cnt = np.zeros(shape, np.int32); np.add.at(cnt, (ij[:, 0], ij[:, 1]), 1)
    wall = cnt >= min_pts
    disk = (np.add.outer(np.arange(-r_close, r_close + 1) ** 2, np.arange(-r_close, r_close + 1) ** 2) <= r_close ** 2)
    closed = ndimage.binary_closing(wall, structure=disk, border_value=0)
    ext = ndimage.binary_propagation(np.pad(np.zeros(np.array(shape) - 2, bool), 1, constant_values=True), mask=~closed)
    inside = ~ext
    # keep only the largest connected interior piece(s) that contain robot paths
    return inside, wall, closed, lo, shape


def building_mask(r_close=10):
    import json
    inside, wall, closed, lo, shape = interior_mask(r_close)
    lab, n = ndimage.label(inside)
    tr = json.load(open('/home/claude/out/viewer_traj.json'))
    keep = set()
    for k in tr:
        t = np.array(tr[k])[:, :2]
        ij = np.floor((t - lo) / RES).astype(int)
        keep |= set(lab[ij[:, 0], ij[:, 1]].tolist())
    keep.discard(0)
    bm = np.isin(lab, list(keep))
    bm = ndimage.binary_dilation(bm, iterations=2)       # 20 cm margin for wall surfaces
    return bm, lo, shape


def classify(points_xy, bm, lo, shape):
    ij = np.floor((points_xy - lo) / RES).astype(int)
    ok = (ij >= 0).all(1) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    res = np.zeros(len(points_xy), bool); res[ok] = bm[ij[ok, 0], ij[ok, 1]]
    return res


def reach_mask(r_close=10, robot_r=4):
    """Free space reachable from the robot paths without crossing (closed) walls."""
    import json
    inside, wall, closed, lo, shape = interior_mask(r_close)
    tr = json.load(open('/home/claude/out/viewer_traj.json'))
    seed = np.zeros(shape, bool)
    for k in tr:
        t = np.array(tr[k])[:, :2]
        # densify path
        tt = np.vstack([np.linspace(t[i], t[i + 1], 10) for i in range(len(t) - 1)])
        ij = np.floor((tt - lo) / RES).astype(int); seed[ij[:, 0], ij[:, 1]] = True
    path = ndimage.binary_dilation(seed, iterations=robot_r)
    free = ~closed | path
    V = ndimage.binary_propagation(seed, mask=free)
    return V, closed, wall, lo, shape, path


def final_mask(grow_cells=12):
    V, closed, wall, lo, shape, path = reach_mask(10)
    M = ndimage.binary_fill_holes(ndimage.binary_dilation(V, iterations=grow_cells))
    return M, lo, shape


# user-confirmed indoor areas (museum frame, metres) added on top of the automatic mask
EXTRA_INDOOR = {
    'north_area_by_glass_door': [(0.5, 1.6), (0.5, 7.0), (-1.6, 7.0), (-1.6, 10.9), (5.2, 10.9), (9.5, 9.3), (9.5, 3.0), (6.0, 3.3), (3.0, 2.5)],
    'ne_walkway_with_obstacles': [(16.0, 1.6), (16.6, 4.6), (21.0, 7.0), (23.0, 10.2), (25.2, 11.4), (27.2, 10.9), (26.5, 9.6), (24.0, 7.0), (18.0, 2.0)],
}


def final_mask_v2():
    from matplotlib.path import Path
    M, lo, shape = final_mask(12)
    gx, gy = np.meshgrid((np.arange(shape[0]) + .5) * RES + lo[0], (np.arange(shape[1]) + .5) * RES + lo[1], indexing='ij')
    G = np.stack([gx.ravel(), gy.ravel()], 1)
    for poly in EXTRA_INDOOR.values():
        M |= Path(poly).contains_points(G).reshape(shape)
    return M, lo, shape
