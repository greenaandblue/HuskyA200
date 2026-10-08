# Data Manifest

[中文](DATA_MANIFEST.zh-CN.md) | **English**

> This repository is a **temporary shared entry point** for robot-collected data, for the reconstruction team (Johann, Ying, et al.) and the digital-twin team.
> Long-term storage, compute, and publishing will be decided after discussion with ICDS.
> Last updated: 2026-10-08 | Maintainer: Huihang Liu

## Status legend

| Mark | Meaning |
|---|---|
| ✅ Available | Collected and checked; ready to use |
| ⚠️ Unverified | Collected, but accuracy or correctness not yet checked |
| ❌ Missing | Not available yet; needs to be collected or computed |

## Sample overview

| Sample folder | Robot | Date | Location | Duration | Contents | Poses | Calibration |
|---|---|---|---|---|---|---|---|
| `samples/go2_test02/` | Unitree Go2 | ____ | ____ | ____ | RGB, depth, timestamps | ❌ Missing | ⚠️ Intrinsics only / ❌ extrinsics |
| `samples/husky_1008/` | Clearpath Husky A200 | 2026-10-08 | ____ | 414 s (≈160 s driving) | LiDAR point clouds, TF, odometry, AMCL pose, 2D map, 3D point-cloud map | ⚠️ KISS-ICP loop error 0.13 m | ✅ LiDAR extrinsics tape-measured |

## Data types

| Data type | Source (robot / sensor / topic) | File format | Units | Frame | Location | Used by | Status | Conversion needed / notes |
|---|---|---|---|---|---|---|---|---|
| RGB images | Go2 / ____ (camera model) | ____ (png / jpg) | pixels | camera optical frame | `go2_test02/rgb/` | Reconstruction | ✅ / ⚠️ | Resolution ____ × ____, frame rate ____ |
| Depth images | Go2 / ____ | ____ (16-bit png) | ____ (mm / m) | camera optical frame | `go2_test02/depth/` | Reconstruction | ✅ / ⚠️ | Aligned to RGB: ____ |
| Timestamps | Go2 / ____ | csv | ____ (s / ns) | — | `go2_test02/timestamps.csv` | Reconstruction | ✅ / ⚠️ | Clock source: ____ |
| Camera intrinsics | Go2 / ____ | yaml | pixels | — | `go2_test02/camera_intrinsics.yaml` | Reconstruction | ✅ / ⚠️ / ❌ | fx, fy, cx, cy, distortion |
| Camera extrinsics (camera → body) | Go2 | yaml | m, rad | `base` → `camera` | ____ | Reconstruction, digital twin | ❌ Missing | To be calibrated |
| Camera poses | Go2 | ____ | m, quaternion | ____ | ____ | Reconstruction | ❌ Missing | Options: Go2 odometry + extrinsics, or estimate with RTAB-Map / COLMAP |
| 3D point clouds (raw) | Husky / Velodyne VLP-16 / `/a200_0544/sensors/lidar3d_0/points` | ROS 2 bag (mcap, 2.5 GB) | m | `lidar3d_0_laser` | [OneDrive](https://pennstateoffice365-my.sharepoint.com/:f:/g/personal/hql5777_psu_edu/IgA_7Cs5Jh-zTJNAbwq7xDVJAbg5i8gbh1v7fchn9oirfLY?e=bYKxtB): `husky_data/husky_1008_bag.zip` | Reconstruction | ✅ | ≈9.9 Hz, 4109 frames |
| TF tree | Husky / `/a200_0544/tf`, `/a200_0544/tf_static` | ROS 2 bag | m, quaternion | multiple | same bag | Reconstruction | ✅ | Includes measured LiDAR extrinsics |
| Wheel odometry | Husky / `/a200_0544/platform/odom/filtered` | ROS 2 bag | m, quaternion | `odom` → `base_link` | same bag | Reconstruction | ⚠️ Unverified | ≈44 Hz; drifts over time |
| Pose in map | Husky / AMCL `/a200_0544/amcl_pose` | ROS 2 bag | m, quaternion | `map` | same bag | Reconstruction, digital twin | ⚠️ Unverified | 2D localization, 199 messages |
| 2D occupancy map | Husky / slam_toolbox | pgm + yaml | 0.05 m / pixel | `map` | `husky_1008/map/` | Robot navigation, digital twin | ✅ | Alignment with building model: ❌ to do |
| 3D point-cloud map | Husky / KISS-ICP (offline) | ply | m | first LiDAR frame | `husky_1008/lab_map_3d.ply` | Reconstruction, digital twin | ✅ | 1.89 M points, 0.05 m voxels; path 59.5 m, loop error 0.13 m |
| 3D poses (per frame) | Husky / KISS-ICP | npy (N×4×4) | m | first LiDAR frame | `husky_1008/poses/` | Reconstruction | ⚠️ Unverified | One pose per point-cloud frame |
| Robot configuration | Husky / `/etc/clearpath/robot.yaml` | yaml | — | — | `husky_1008/robot.yaml` | Everyone | ✅ | — |

## Frame relationships

```
Building model (Unity / Cesium)
        │  ❌ alignment to do (method: TBD with the reconstruction and digital-twin teams)
     map  (room map, built by Husky slam_toolbox)
        │  AMCL localization
     odom
        │  wheel odometry (EKF-filtered)
  base_link  (Husky body)
        │  extrinsics in robot.yaml (measured xyz = [-0.28, 0, 0.748], rpy = 0)
  lidar3d_0_laser  (VLP-16)

  Go2 body ──❌ extrinsics to be calibrated──► depth camera
  Go2 ↔ Husky map relationship: ❌ TBD
```

## To do / to confirm

- [ ] Reconstruction team: can you use the samples directly? What format do you need for poses and calibration?
- [ ] Go2 camera extrinsic calibration
- [ ] Go2 poses: use odometry or estimate afterwards?
- [x] Measure the actual Husky LiDAR mounting position and update extrinsics (2026-10-08)
- [ ] Method for aligning the room map with the building model
- [ ] Upload, retention, and public scope for visitor images (needs a separate decision)

## Access and use

- The repository is **private** by default. Access: Huihang Liu, Johann, Ying; others on request
- GitHub limits single files to 100 MB; raw bags are on Penn State OneDrive: [husky_data](https://pennstateoffice365-my.sharepoint.com/:f:/g/personal/hql5777_psu_edu/IgA_7Cs5Jh-zTJNAbwq7xDVJAbg5i8gbh1v7fchn9oirfLY?e=bYKxtB) (Penn State accounts only)
- Questions: Huihang Liu (hql5777@psu.edu)
