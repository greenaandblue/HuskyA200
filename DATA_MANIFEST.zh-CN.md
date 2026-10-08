# 数据清单 DATA_MANIFEST

**中文** | [English](DATA_MANIFEST.md)

> 本仓库是机器人采集数据的**临时共享入口**，供重建端（Johann、Ying 等）和数字孪生端使用。
> 长期存储、计算和发布方案待与 ICDS 讨论后确定。
> 最后更新：2026-10-08 ｜ 维护人：Huihang Liu

## 状态说明

| 标记 | 含义 |
|---|---|
| ✅ 已有 | 数据已采集并检查过，可以直接使用 |
| ⚠️ 未验证 | 数据已采集，但还没有检查精度或正确性 |
| ❌ 缺失 | 目前还没有，需要补采或另外计算 |

## 样本总览

| 样本目录 | 机器人 | 采集日期 | 地点 | 时长 | 包含数据 | 位姿 | 标定 |
|---|---|---|---|---|---|---|---|
| `samples/go2_test02/` | Unitree Go2 | ____ | ____ | ____ | RGB、深度、时间戳 | ❌ 缺失 | ⚠️ 只有内参 / ❌ 外参 |
| `samples/husky_1008/` | Clearpath Husky A200 | 2026-10-08 | ____ | 414 s（行驶约 160 s） | LiDAR 点云、TF、里程计、AMCL 位姿、2D 地图、3D 点云地图 | ⚠️ KISS-ICP 回环误差 0.13 m | ✅ 雷达外参卷尺实测 |

## 各类数据说明

| 数据类型 | 来源（机器人 / 传感器 / 话题） | 文件格式 | 单位 | 坐标系 | 保存位置 | 使用方 | 状态 | 需要的转换 / 备注 |
|---|---|---|---|---|---|---|---|---|
| RGB 图像 | Go2 / ____（相机型号） | ____（png / jpg） | 像素 | 相机光学坐标系 | `go2_test02/rgb/` | 重建端 | ✅ / ⚠️ | 分辨率 ____ × ____，帧率 ____ |
| 深度图像 | Go2 / ____ | ____（16 位 png） | ____（mm / m） | 相机光学坐标系 | `go2_test02/depth/` | 重建端 | ✅ / ⚠️ | 是否已对齐到 RGB：____ |
| 时间戳 | Go2 / ____ | csv | ____（s / ns） | — | `go2_test02/timestamps.csv` | 重建端 | ✅ / ⚠️ | 时钟来源：____ |
| 相机内参 | Go2 / ____ | yaml | 像素 | — | `go2_test02/camera_intrinsics.yaml` | 重建端 | ✅ / ⚠️ / ❌ | fx, fy, cx, cy, 畸变系数 |
| 相机外参（相机 → 机身） | Go2 | yaml | m, rad | `base` → `camera` | ____ | 重建端、数字孪生端 | ❌ 缺失 | 待标定 |
| 相机位姿 | Go2 | ____ | m, 四元数 | ____ | ____ | 重建端 | ❌ 缺失 | 可选方案：Go2 里程计 + 外参，或用 RTAB-Map / COLMAP 估计 |
| 3D 点云（原始） | Husky / Velodyne VLP-16 / `/a200_0544/sensors/lidar3d_0/points` | ROS 2 bag（mcap，2.5 GB） | m | `lidar3d_0_laser` | [OneDrive](https://pennstateoffice365-my.sharepoint.com/:f:/g/personal/hql5777_psu_edu/IgA_7Cs5Jh-zTJNAbwq7xDVJAbg5i8gbh1v7fchn9oirfLY?e=bYKxtB)：`husky_data/husky_1008_bag.zip` | 重建端 | ✅ | 约 9.9 Hz，4109 帧 |
| TF 树 | Husky / `/a200_0544/tf`, `/a200_0544/tf_static` | ROS 2 bag | m, 四元数 | 多个 | 同上 | 重建端 | ✅ | 包含雷达外参（实测） |
| 轮式里程计 | Husky / `/a200_0544/platform/odom/filtered` | ROS 2 bag | m, 四元数 | `odom` → `base_link` | 同上 | 重建端 | ⚠️ 未验证 | 约 44 Hz，会随时间漂移 |
| 地图内位姿 | Husky / AMCL `/a200_0544/amcl_pose` | ROS 2 bag | m, 四元数 | `map` | 同上 | 重建端、数字孪生端 | ⚠️ 未验证 | 2D 定位，199 条 |
| 2D 栅格地图 | Husky / slam_toolbox | pgm + yaml | 0.05 m / 像素 | `map` | `husky_1008/map/` | 机器人导航、数字孪生端 | ✅ | 与建筑模型的对齐：❌ 待做 |
| 3D 点云地图 | Husky / KISS-ICP 离线计算 | ply | m | 第一帧雷达坐标系 | `husky_1008/lab_map_3d.ply` | 重建端、数字孪生端 | ✅ | 189 万点，体素 0.05 m；路程 59.5 m，回环误差 0.13 m |
| 3D 位姿（逐帧） | Husky / KISS-ICP | npy（N×4×4） | m | 第一帧雷达坐标系 | `husky_1008/poses/` | 重建端 | ⚠️ 未验证 | 与点云逐帧对应 |
| 机器人配置 | Husky / `/etc/clearpath/robot.yaml` | yaml | — | — | `husky_1008/robot.yaml` | 所有人 | ✅ | — |

## 坐标系关系

```
建筑模型（Unity / Cesium）
        │  ❌ 待对齐（方法：待与重建端、数字孪生端商定）
     map（房间地图，Husky slam_toolbox 生成）
        │  AMCL 定位
     odom
        │  轮式里程计（EKF 滤波）
  base_link（Husky 车身）
        │  robot.yaml 中的外参（实测 xyz = [-0.28, 0, 0.748]，rpy = 0）
  lidar3d_0_laser（VLP-16）

  Go2 机身 ──❌ 外参待标定──► 深度相机
  Go2 与 Husky 地图的关系：❌ 待定
```

## 待办与待确认

- [ ] 请重建端确认：样本能否直接使用？位姿和标定需要什么格式？
- [ ] Go2 相机外参标定
- [ ] Go2 位姿：用里程计还是事后估计？
- [x] 测量 Husky 雷达的真实安装位置，更新外参（2026-10-08）
- [ ] 房间地图与建筑模型的对齐方式
- [ ] 游客图像的上传、保留和公开范围（需要单独确认）

## 访问与使用

- 仓库默认**私有**，访问范围：Huihang Liu、Johann、Ying，其他人按需添加
- GitHub 单个文件上限 100 MB；原始 bag 放在 Penn State OneDrive：[husky_data](https://pennstateoffice365-my.sharepoint.com/:f:/g/personal/hql5777_psu_edu/IgA_7Cs5Jh-zTJNAbwq7xDVJAbg5i8gbh1v7fchn9oirfLY?e=bYKxtB)（需要 Penn State 账号）
- 有问题请联系：Huihang Liu（hql5777@psu.edu）
