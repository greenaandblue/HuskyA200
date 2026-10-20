"""Build a lidar map for one Husky bag with KISS-ICP.

Saves <out>.npz with:
  poses   (N,4,4)  lidar pose in KISS map frame (first scan = identity)
  stamps  (N,)     scan header stamps [s]
  odom    (N,4,4)  odom->base_link (wheel/EKF) interpolated at scan stamps
  pts     (M,4)    deskewed scan points in lidar frame, 5 cm voxelised: x,y,z,intensity
  idx     (N+1,)   offsets into pts per scan
"""
import sys, time
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation as R, Slerp
import open3d as o3d
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP

bag, out = Path(sys.argv[1]), sys.argv[2]
NS = '/a200_0544'
ts = get_typestore(Stores.ROS2_JAZZY)

cfg = KISSConfig()
cfg.data.max_range = 100.0   # range filtering done below so point order is preserved
cfg.data.min_range = 0.0
cfg.data.deskew = True
cfg.mapping.voxel_size = 0.2
cfg.mapping.max_points_per_voxel = 20
def planar(T):
    """Project a pose onto SE(2): keep x, y, yaw; zero z, roll, pitch (flat museum floor)."""
    yaw = np.arctan2(T[1, 0], T[0, 0])
    P = np.eye(4); c, s = np.cos(yaw), np.sin(yaw)
    P[:2, :2] = [[c, -s], [s, c]]; P[:2, 3] = T[:2, 3]
    return P


class PlanarKissICP(KissICP):
    def register_frame(self, frame, timestamps):
        frame = self.preprocessor.preprocess(frame, timestamps, self.last_delta)
        source, frame_downsample = self.voxelize(frame)
        sigma = self.adaptive_threshold.get_threshold()
        initial_guess = self.last_pose @ self.last_delta
        new_pose = self.registration.align_points_to_map(
            points=source, voxel_map=self.local_map, initial_guess=initial_guess,
            max_correspondance_distance=3 * sigma, kernel=sigma)
        new_pose = planar(new_pose)
        self.adaptive_threshold.update_model_deviation(np.linalg.inv(initial_guess) @ new_pose)
        self.local_map.update(frame_downsample, new_pose)
        self.last_delta = np.linalg.inv(self.last_pose) @ new_pose
        self.last_pose = new_pose
        return frame, source


odom_icp = PlanarKissICP(cfg) if '--3d' not in sys.argv else KissICP(cfg)

dt = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
               'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
               'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 22})

poses, stamps, pts, idx = [], [], [], [0]
otimes, opos, oquat = [], [], []
t0 = time.time()
with AnyReader([bag], default_typestore=ts) as r:
    conns = [c for c in r.connections if c.topic in (f'{NS}/sensors/lidar3d_0/points', f'{NS}/tf')]
    for c, t, raw in r.messages(connections=conns):
        m = r.deserialize(raw, c.msgtype)
        if c.topic.endswith('/tf'):
            for tr in m.transforms:
                if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                    st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                    p, q = tr.transform.translation, tr.transform.rotation
                    otimes.append(st); opos.append((p.x, p.y, p.z)); oquat.append((q.x, q.y, q.z, q.w))
            continue
        a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
        xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        rng = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(xyz).all(1) & (rng > 1.0) & (rng < 40.0)  # drop robot body / operator
        xyz, tt, inten = xyz[ok], a['time'][ok].astype(np.float64), a['intensity'][ok]
        if len(xyz) < 500:
            continue
        frame, _ = odom_icp.register_frame(xyz, tt)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        poses.append(odom_icp.last_pose.copy()); stamps.append(st)
        # keep deskewed frame (sensor frame) at 5 cm; intensity via nearest original index
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(frame))
        assert len(frame) == len(inten)
        pc.colors = o3d.utility.Vector3dVector(np.repeat(inten[:, None], 3, 1).astype(np.float64) / 255.0)
        pc = pc.voxel_down_sample(0.05)
        f = np.asarray(pc.points, np.float32)
        ii = np.asarray(pc.colors, np.float32)[:, :1] * 255
        pts.append(np.hstack([f, ii])); idx.append(idx[-1] + len(f))
        if len(poses) % 200 == 0:
            p = odom_icp.last_pose[:3, 3]
            print(f'{len(poses)} scans  {time.time()-t0:.0f}s  pos=({p[0]:.1f},{p[1]:.1f},{p[2]:.2f})', flush=True)

stamps = np.array(stamps); otimes = np.array(otimes)
o = np.argsort(otimes); otimes = otimes[o]; opos = np.array(opos)[o]; oquat = np.array(oquat)[o]
otimes, u = np.unique(otimes, return_index=True); opos, oquat = opos[u], oquat[u]
sc = np.clip(stamps, otimes[0], otimes[-1])
slerp = Slerp(otimes, R.from_quat(oquat))
odom = np.tile(np.eye(4), (len(stamps), 1, 1))
odom[:, :3, :3] = slerp(sc).as_matrix()
for k in range(3):
    odom[:, k, 3] = np.interp(sc, otimes, opos[:, k])
np.savez_compressed(out, poses=np.array(poses), stamps=stamps, odom=odom,
                    pts=np.vstack(pts), idx=np.array(idx))
print('done', len(poses), 'scans', idx[-1], 'pts', f'{time.time()-t0:.0f}s')
