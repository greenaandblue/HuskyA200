"""Helpers: assemble a room map from saved scans and render top-down / side views."""
import numpy as np, open3d as o3d
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

LIDAR_IN_BASE = np.eye(4); LIDAR_IN_BASE[:3, 3] = [-0.28, 0.0, 0.748 + 0.0377]


def load(name):
    return dict(np.load(f'/home/claude/work/{name}.npz'))


def assemble(d, poses=None, step=1, voxel=0.05, zmax=None):
    poses = d['poses'] if poses is None else poses
    idx, pts = d['idx'], d['pts']
    out = []
    for i in range(0, len(poses), step):
        p = pts[idx[i]:idx[i + 1]]
        w = p[:, :3] @ poses[i][:3, :3].T + poses[i][:3, 3]
        out.append(np.hstack([w, p[:, 3:4]]))
    a = np.vstack(out).astype(np.float64)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(a[:, :3]))
    pc.colors = o3d.utility.Vector3dVector(np.repeat(a[:, 3:4] / 255.0, 3, 1))
    return pc.voxel_down_sample(voxel)


def topdown(clouds, path, title='', zrange=(0.3, 2.0), trajs=(), size=10, lim=None):
    """clouds: list of (o3d cloud, color). Points in sensor-height frame: z relative to lidar."""
    fig, ax = plt.subplots(figsize=(size, size))
    for pc, col in clouds:
        a = np.asarray(pc.points)
        m = (a[:, 2] > zrange[0]) & (a[:, 2] < zrange[1])
        ax.scatter(a[m, 0], a[m, 1], s=0.05, c=col, alpha=0.4, linewidths=0)
    for t, col in trajs:
        ax.plot(t[:, 0], t[:, 1], '-', c=col, lw=1)
        ax.plot(t[0, 0], t[0, 1], 'o', c=col)
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_title(title)
    if lim: ax.set_xlim(lim[0]); ax.set_ylim(lim[1])
    fig.savefig(path, dpi=110, bbox_inches='tight'); plt.close(fig)
