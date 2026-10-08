#!/usr/bin/env python3
"""
把 KISS-ICP 算出的位姿和 rosbag 里的点云拼成一整张 3D 点云地图（.ply）。

用法：
    python build_map.py <bag 目录> <poses.npy> [选项]

例子：
    python build_map.py lab_3d results/latest/lab_3d_poses.npy

选项：
    --topic       点云话题（默认 /a200_0544/sensors/lidar3d_0/points）
    --voxel       体素大小，单位米，越小越细但文件越大（默认 0.05）
    --min-range   去掉离雷达太近的点，比如车身（默认 1.0 米）
    --max-range   去掉太远、噪声大的点（默认 40 米）
    --out         输出文件名（默认 lab_map_3d.ply）
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def cloud_to_xyz(msg):
    """PointCloud2 -> N x 3 的 numpy 数组（只取 x, y, z）。"""
    offsets = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4"] * 3,
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    buf = np.asarray(msg.data, dtype=np.uint8).tobytes()
    pts = np.frombuffer(buf, dtype=dtype, count=n)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_down(points, voxel):
    """每个体素格子里只留一个点。"""
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


def write_ply(path, points):
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(points.astype("<f4").tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("poses")
    ap.add_argument("--topic", default="/a200_0544/sensors/lidar3d_0/points")
    ap.add_argument("--voxel", type=float, default=0.05)
    ap.add_argument("--min-range", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--out", default="lab_map_3d.ply")
    args = ap.parse_args()

    poses = np.load(args.poses)
    print(f"读取到 {len(poses)} 个位姿")

    merged = np.empty((0, 3))
    chunk = []
    used = 0

    with AnyReader([Path(args.bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            print(f"bag 里没有话题 {args.topic}，现有的话题：")
            for c in reader.connections:
                print("  ", c.topic)
            sys.exit(1)

        for conn, _, raw in reader.messages(connections=conns):
            if used >= len(poses):
                break
            msg = reader.deserialize(raw, conn.msgtype)
            xyz = cloud_to_xyz(msg)
            r = np.linalg.norm(xyz, axis=1)
            xyz = xyz[(r > args.min_range) & (r < args.max_range)]

            T = poses[used]
            chunk.append(xyz @ T[:3, :3].T + T[:3, 3])
            used += 1

            if len(chunk) >= 50:
                merged = voxel_down(np.vstack([merged] + chunk), args.voxel)
                chunk = []
                print(f"  已处理 {used} 帧，地图 {len(merged):,} 个点")

    if chunk:
        merged = voxel_down(np.vstack([merged] + chunk), args.voxel)

    if used != len(poses):
        print(f"注意：用了 {used} 帧，但位姿有 {len(poses)} 个，对不上的话地图可能会错位")

    write_ply(args.out, merged)
    print(f"完成！{len(merged):,} 个点已保存到 {args.out}")


if __name__ == "__main__":
    main()
