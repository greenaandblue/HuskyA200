"""Prepare the cleaned point cloud for meshing: inside-only, no reflections, no door leaves, oriented normals."""
import json, numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from interior import classify, RES

D = '/mnt/user-data/outputs/museum_merged'
mk = np.load('/home/claude/work/final_mask.npz'); M, lo, shape = mk['M'], mk['lo'], tuple(mk['shape'])

def read_ply(p):
    s = open(p, 'rb').read(); h = s.index(b'end_header\n') + 11
    r = np.frombuffer(s[h:], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', 'u1')])
    return np.stack([r['x'], r['y'], r['z']], 1).astype(np.float64), r['i']

P, I, R = [], [], []
for r, k in enumerate(['room01', 'room04', 'room05']):
    p, i = read_ply(f'{D}/{k}_aligned.ply'); P.append(p); I.append(i); R.append(np.full(len(p), r, np.int8))
P = np.vstack(P); I = np.concatenate(I); R = np.concatenate(R)
n0 = len(P)
keep = classify(P[:, :2], M, lo, shape) & (P[:, 2] > -0.12)

# door leaves (modelled separately): measured open position, hall side of the wall
DOOR = json.load(open('/home/claude/work/door.json'))
for leaf in DOOR['leaves']:
    hx, hy = leaf['hinge']; L = DOOR['leaf_width']
    box = (P[:, 0] > hx - 0.02) & (P[:, 0] < hx + L + 0.15) & (np.abs(P[:, 1] - leaf['open_y']) < 0.13) & (P[:, 2] < DOOR['height'] + 0.08)
    keep &= ~box
P, I, R = P[keep], I[keep], R[keep]
# one recording per 0.4 m voxel: removes 5-10 cm double walls from per-recording drift
vox = np.floor(P / 0.4).astype(np.int64)
key = (vox[:, 0] + 10000) * 100000000 + (vox[:, 1] + 10000) * 10000 + (vox[:, 2] + 100)
uk, inv = np.unique(key, return_inverse=True)
cnt = np.zeros((len(uk), 3), np.int64); np.add.at(cnt, (inv, R), 1)
owner = cnt.argmax(1)
own = owner[inv] == R
P, I, R = P[own], I[own], R[own]
print('points', n0, '->', len(P), '(single-recording per voxel)')

pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
pc.colors = o3d.utility.Vector3dVector(np.repeat(I[:, None] / 255.0, 3, 1))
pc = pc.voxel_down_sample(0.04)
pc, _ = pc.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.5)
pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.22, max_nn=40))

# orient normals toward the sensor: nearest robot path sample (lidar height 0.92 m)
tr = json.load(open('/home/claude/out/viewer_traj.json'))
S = np.vstack([np.array(tr[k]) for k in tr]); S[:, 2] = 0.92
pts = np.asarray(pc.points); nrm = np.asarray(pc.normals)
_, j = cKDTree(S).query(pts, k=1)
flip = np.einsum('ij,ij->i', nrm, S[j] - pts) < 0
nrm[flip] *= -1
# floor normals must point up, ceilings down (fix the near-horizontal-ray ambiguity)
fl = (pts[:, 2] < 0.08) & (np.abs(nrm[:, 2]) > 0.8); nrm[fl] *= np.sign(nrm[fl][:, 2:3])
pc.normals = o3d.utility.Vector3dVector(nrm)
o3d.io.write_point_cloud('/home/claude/work/mesh_input.ply', pc)
print('mesh input points', len(pc.points))
