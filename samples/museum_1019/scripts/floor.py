"""Flat floor slab from lidar free space (rays passing over the floor) + observed floor points."""
import json, numpy as np, trimesh
from scipy import ndimage
from viz import load
from freespace import scan_rays, grids

mk = np.load('/home/claude/work/final_mask.npz'); M, lo, shape = mk['M'], mk['lo'], tuple(mk['shape'])
RES = 0.1
T = json.load(open('/mnt/user-data/outputs/museum_merged/transforms.json'))
occ = np.zeros(shape, np.float32); free = np.zeros(shape, np.float32)
for k in ['room01', 'room04', 'room05']:
    d = load(k)
    rays = scan_rays(d, np.array(T[k]), every=6, nray=2500)
    o, f = grids(rays, RES, lo, shape)
    occ = np.maximum(occ, o); free = np.maximum(free, f)
fp = np.load('/home/claude/work/floor_pts.npy')
ij = np.floor((fp - lo) / RES).astype(int)
ok = (ij >= 0).all(1) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
seen = np.zeros(shape, bool); seen[ij[ok, 0], ij[ok, 1]] = True
F = (free > 0) | seen
F &= M
F = ndimage.binary_closing(F, iterations=2)
# fill holes up to 3 m^2 (under exhibits, around the robot's own footprint)
holes = ndimage.binary_fill_holes(F) & ~F
lab, n = ndimage.label(holes)
if n:
    sz = ndimage.sum(holes, lab, np.arange(1, n + 1))
    F |= np.isin(lab, np.where(sz * RES * RES <= 3.0)[0] + 1)
lab, n = ndimage.label(F)
sz = ndimage.sum(F, lab, np.arange(1, n + 1))
F = np.isin(lab, np.where(sz * RES * RES >= 1.0)[0] + 1)
print('floor area m2', round(F.sum() * RES * RES))
np.save('/home/claude/work/floor_mask.npy', F)

# triangulate: merge each row's runs into rectangles (keeps the triangle count low)
V, Fc = [], []
for i in range(shape[0]):
    row = F[i]
    if not row.any():
        continue
    d = np.diff(np.concatenate([[0], row.astype(int), [0]]))
    starts, ends = np.where(d == 1)[0], np.where(d == -1)[0]
    x0, x1 = lo[0] + i * RES, lo[0] + (i + 1) * RES
    for s, e in zip(starts, ends):
        y0, y1 = lo[1] + s * RES, lo[1] + e * RES
        b = len(V)
        V += [[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]]
        Fc += [[b, b + 1, b + 2], [b, b + 2, b + 3]]
floor = trimesh.Trimesh(np.array(V), np.array(Fc), process=True)
floor.export('/home/claude/work/floor_mesh.ply')
print('floor tris', len(floor.faces))
