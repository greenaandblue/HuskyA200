"""Assemble the merged museum map: point clouds, Nav2 occupancy map, floor-plan image, viewer data."""
import json, numpy as np, open3d as o3d
from scipy import ndimage
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from viz import load, LIDAR_IN_BASE
from freespace import scan_rays
from register import se2

OUT = '/home/claude/out'
import os; os.makedirs(OUT, exist_ok=True)
FLOOR = -1.03            # floor height in lidar-start frame (measured)
names = {'room01': 'hall (room01)', 'room04': 'room04', 'room05': 'room05'}
d = {k: load(k) for k in names}

# --- transforms into the room04 frame ---
T45 = np.load('T_04_05.npy'); T401 = np.load('T_45_01_c0.npy')
T = {'room04': np.eye(4), 'room05': T45, 'room01': T401}

# --- global frame: walls axis-aligned, floor at z=0, origin at room04 start (hall doorway) ---
wall_pts = []
for k in ['room04', 'room05']:   # rectilinear rooms define the building axes
    p = d[k]['pts'][::50, :3].astype(np.float64)
    wall_pts.append(p @ T[k][:3, :3].T + T[k][:3, 3])
pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(wall_pts)))
pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30))
n = np.asarray(pc.normals); n = n[np.abs(n[:, 2]) < 0.2]
ang = np.mod(np.arctan2(n[:, 1], n[:, 0]), np.pi / 2)
h, e = np.histogram(ang, 180, (0, np.pi / 2)); a0 = e[np.argmax(ndimage.uniform_filter1d(h, 5, mode='wrap'))] + np.pi / 360
a0 = a0 - np.pi / 2 if a0 > np.pi / 4 else a0
G = se2(0, 0, -a0); G[2, 3] = -FLOOR
for k in T:
    T[k] = G @ T[k]
print('wall rotation applied (deg):', np.degrees(a0).round(2))


# --- dynamic-object removal: 2D static ratio from all rays, only near trajectories ---
res = 0.1
rays = sum([scan_rays(d[k], T[k], every=8, nray=2000) for k in names], [])
allp = np.vstack([hh for _, hh in rays]); lo = allp.min(0) - 1; hi = allp.max(0) + 1
shape = tuple(np.ceil((hi - lo) / res).astype(int))
occ = np.zeros(shape, np.int32); free = np.zeros(shape, np.int32)
for o, hh in rays:
    v = hh - o; L = np.linalg.norm(v, axis=1); m = int(np.ceil(L.max() / (res * .5)))
    s = np.linspace(0, 1, m, endpoint=False)[:, None, None]
    q = (o + s * v[None])[(s[..., 0] * L[None]) < L[None] - 2 * res]
    ij = np.floor((q - lo) / res).astype(int); free += np.bincount(ij[:, 0] * shape[1] + ij[:, 1], minlength=free.size).reshape(shape).astype(np.int32)
    ij = np.floor((hh - lo) / res).astype(int); np.add.at(occ, (ij[:, 0], ij[:, 1]), 1)
ratio = occ / np.maximum(occ + free, 1)
dynamic = (free >= 4) & (ratio < 0.12)
traj = np.zeros(shape, bool)
for k in names:
    tp = np.array([(T[k] @ P)[:2, 3] for P in d[k]['poses']])
    ij = np.floor((tp - lo) / res).astype(int); traj[ij[:, 0], ij[:, 1]] = True
near_traj = ndimage.binary_dilation(traj, iterations=25)      # 2.5 m
dyn_mask = dynamic & near_traj


def assemble(k, voxel=0.04, chunk=250):
    P, idx, pts = d[k]['poses'], d[k]['idx'], d[k]['pts']
    acc = o3d.geometry.PointCloud(); nbad = 0
    for c0 in range(0, len(P), chunk):
        out = []
        for i in range(c0, min(c0 + chunk, len(P))):
            p = pts[idx[i]:idx[i + 1]]
            M = (T[k] @ P[i]).astype(np.float32)
            out.append(np.hstack([p[:, :3] @ M[:3, :3].T + M[:3, 3], p[:, 3:4]]))
        a = np.vstack(out); del out
        ij = np.clip(np.floor((a[:, :2] - lo) / res).astype(int), 0, np.array(shape) - 1)
        bad = dyn_mask[ij[:, 0], ij[:, 1]] & (a[:, 2] > 0.08) & (a[:, 2] < 2.1)
        nbad += int(bad.sum()); a = a[~bad]
        c = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(a[:, :3].astype(np.float64)))
        c.colors = o3d.utility.Vector3dVector(np.repeat(np.clip(a[:, 3:4] / 100.0, 0, 1), 3, 1).astype(np.float64))
        acc += c.voxel_down_sample(voxel); del a, c
        acc = acc.voxel_down_sample(voxel)
    acc, _ = acc.remove_radius_outlier(nb_points=4, radius=0.2)
    print(k, 'removed dynamic', nbad, '-> points', len(acc.points), flush=True)
    return acc


clouds = {k: assemble(k) for k in names}
for k, c in clouds.items():
    o3d.io.write_point_cloud(f'{OUT}/{k}_aligned.ply', c)
merged = clouds['room01'] + clouds['room04'] + clouds['room05']
merged = merged.voxel_down_sample(0.04)
o3d.io.write_point_cloud(f'{OUT}/museum_merged.ply', merged)
o3d.io.write_point_cloud(f'{OUT}/museum_merged.pcd', merged)
print('merged points', len(merged.points))

# --- transforms file ---
with open(f'{OUT}/transforms.json', 'w') as f:
    json.dump({'frame': 'museum (walls axis-aligned, z=0 floor, origin = room04 start at hall doorway)',
               'note': 'T maps each room KISS-ICP frame (lidar pose at first scan) into the museum frame',
               **{k: T[k].round(6).tolist() for k in T}}, f, indent=1)

# --- Nav2 occupancy map (5 cm) ---
r2 = 0.05
a = np.asarray(merged.points)
sl = a[(a[:, 2] > 0.15) & (a[:, 2] < 1.8)]
lo2 = a[:, :2].min(0) - 1; hi2 = a[:, :2].max(0) + 1
sh2 = tuple(np.ceil((hi2 - lo2) / r2).astype(int))
oc = np.zeros(sh2, np.int32); ij = np.floor((sl[:, :2] - lo2) / r2).astype(int); np.add.at(oc, (ij[:, 0], ij[:, 1]), 1)
OCC = oc >= 3
# free from the 10 cm ray grid, upsampled
fx = ((np.arange(sh2[0]) + .5) * r2 + lo2[0] - lo[0]) / res; fy = ((np.arange(sh2[1]) + .5) * r2 + lo2[1] - lo[1]) / res
FX, FY = np.meshgrid(np.clip(fx.astype(int), 0, shape[0] - 1), np.clip(fy.astype(int), 0, shape[1] - 1), indexing='ij')
FREE = (free[FX, FY] >= 2) & ~OCC
img = np.full(sh2, 205, np.uint8); img[FREE] = 254; img[OCC] = 0
pgm = np.flipud(img.T)
with open(f'{OUT}/museum_map.pgm', 'wb') as f:
    f.write(f'P5\n{pgm.shape[1]} {pgm.shape[0]}\n255\n'.encode()); f.write(pgm.tobytes())
with open(f'{OUT}/museum_map.yaml', 'w') as f:
    f.write(f'image: museum_map.pgm\nmode: trinary\nresolution: {r2}\norigin: [{lo2[0]:.3f}, {lo2[1]:.3f}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

# --- floor plan image coloured by recording ---
cols = {'room01': '#d1495b', 'room04': '#2e86ab', 'room05': '#3a9d5d'}
fig, ax = plt.subplots(figsize=(16, 12))
ax.imshow(np.where(FREE.T, 0.96, 1.0), origin='lower', cmap='gray', vmin=0, vmax=1,
          extent=[lo2[0], lo2[0] + sh2[0] * r2, lo2[1], lo2[1] + sh2[1] * r2])
for k in ['room01', 'room04', 'room05']:
    b = np.asarray(clouds[k].points); b = b[(b[:, 2] > 0.15) & (b[:, 2] < 1.8)]
    ax.scatter(b[:, 0], b[:, 1], s=0.08, c=cols[k], alpha=0.5, linewidths=0, label=names[k])
    tp = np.array([(T[k] @ P)[:2, 3] for P in d[k]['poses']])
    ax.plot(tp[:, 0], tp[:, 1], '--', c=cols[k], lw=0.8, alpha=0.8)
lg = ax.legend(markerscale=60, loc='upper left', fontsize=12)
ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(alpha=0.25)
ax.set_title('Museum — merged Husky lidar map (room01 hall + room04 + room05)', fontsize=14)
fig.savefig(f'{OUT}/museum_floorplan.png', dpi=130, bbox_inches='tight'); plt.close(fig)

# --- viewer data: int16 cm positions + intensity + room id ---
vox = merged.voxel_down_sample(0.08)
parts = []
for rid, k in enumerate(['room01', 'room04', 'room05']):
    c = clouds[k].voxel_down_sample(0.08)
    a = np.asarray(c.points); it = (np.asarray(c.colors)[:, 0] * 255).astype(np.uint8)
    parts.append((np.round(a * 100).astype(np.int16), it, np.full(len(a), rid, np.uint8)))
P = np.vstack([p for p, _, _ in parts]); I = np.concatenate([i for _, i, _ in parts]); R = np.concatenate([r for _, _, r in parts])
with open(f'{OUT}/viewer_points.bin', 'wb') as f:
    f.write(np.array([len(P)], np.uint32).tobytes()); f.write(P.tobytes()); f.write(I.tobytes()); f.write(R.tobytes())
trajs = {k: np.array([(T[k] @ P_)[:3, 3] for P_ in d[k]['poses'][::10]]).round(2).tolist() for k in names}
json.dump(trajs, open(f'{OUT}/viewer_traj.json', 'w'))
print('viewer points', len(P), 'bytes', os.path.getsize(f'{OUT}/viewer_points.bin'))
