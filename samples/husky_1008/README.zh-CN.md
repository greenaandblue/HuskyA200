# 样本：husky_1008

**中文** | [English](README.md)

## 一句话说明

Clearpath Husky A200 搭载 Velodyne VLP-16 在 ____（房间号）采集的 LiDAR 样例，包含原始点云、TF、里程计、AMCL 地图内位姿、2D 栅格地图，以及 KISS-ICP 生成的 3D 点云地图。**雷达外参为卷尺实测值（约 ±1 cm）；KISS-ICP 回环误差 0.13 m。**

![3D 点云地图（俯视）](map_3d_top.png)
![3D 点云地图（斜视）](map_3d_view.png)

## 采集信息

| 项目 | 内容 |
|---|---|
| 日期 / 时间 | 2026-10-08 14:52–14:59 EDT（18:52–18:59 UTC） |
| 地点 | ____（房间号 / 展区） |
| 机器人 | Clearpath Husky A200，序列号 a200-0544，ROS 命名空间 `a200_0544` |
| 系统 | Ubuntu 24.04 Server + ROS 2 Jazzy |
| 雷达 | Velodyne VLP-16，IP 192.168.131.20，约 10 Hz |
| 雷达外参（`base_link` → `lidar3d_0_link`） | xyz = [-0.28, 0.0, 0.748] m，rpy = [0, 0, 0]；卷尺实测，约 ±1 cm |
| 运动方式 | Logitech F710 手柄遥控，绕房间一圈后回到起点；平均速度约 0.37 m/s |
| 时长 | 总计 414 s；实际行驶约 160 s（大约第 145–305 秒），首尾为静止 |
| 采集人 | Huihang Liu |

### 外参测量方法

- `base_link` 位于前后轮轴中点，离地约 0.132 m（轮半径 0.165 m − 轮轴相对 base_link 的高度 0.033 m，后者来自 TF）。
- 雷达底面离地 0.88 m，因此 z = 0.88 − 0.132 = 0.748 m（`lidar3d_0_link` 为雷达底面；激光中心 `lidar3d_0_laser` 在其上方约 3.77 cm）。
- 雷达位于车身中心线上（y = 0），在车身中心后方 0.28 m（x = −0.28）。

## 录制命令

```bash
ros2 bag record -o ~/bags/husky_1008 \
  /a200_0544/sensors/lidar3d_0/points \
  /a200_0544/tf /a200_0544/tf_static \
  /a200_0544/platform/odom/filtered \
  /a200_0544/amcl_pose
```

## 目录结构

```
husky_1008/
├── map/
│   ├── lab_map.pgm       # 2D 栅格地图（slam_toolbox，2026-09-29）
│   └── lab_map.yaml
├── poses/                # KISS-ICP 输出的位姿
├── lab_map_3d.ply        # 3D 点云地图
├── map_3d_top.png        # 截图
├── map_3d_view.png
├── robot.yaml            # 录制时的 /etc/clearpath/robot.yaml
├── build_map.py          # 用 bag + 位姿生成 .ply 的脚本
├── kiss.yaml             # KISS-ICP 配置
├── README.md             # 英文版
└── README.zh-CN.md       # 中文版
```

**原始 bag（2.5 GB）不在仓库里**，请从 [Penn State OneDrive](https://pennstateoffice365-my.sharepoint.com/:f:/g/personal/hql5777_psu_edu/IgA_7Cs5Jh-zTJNAbwq7xDVJAbg5i8gbh1v7fchn9oirfLY?e=bYKxtB) 下载 `husky_data/husky_1008_bag.zip`（需要 Penn State 账号）。同一文件夹里还有 `husky_data/map_3d_video.mov`，是 3D 地图的浏览视频。

## bag 内容

| 话题 | 消息类型 | 消息数 | 频率 | 坐标系 | 状态 |
|---|---|---|---|---|---|
| `/a200_0544/sensors/lidar3d_0/points` | `sensor_msgs/PointCloud2` | 4109 | 约 9.9 Hz | `lidar3d_0_laser` | ✅ |
| `/a200_0544/tf` | `tf2_msgs/TFMessage` | 25457 | — | — | ✅ |
| `/a200_0544/tf_static` | `tf2_msgs/TFMessage` | 1 | — | 含雷达外参 | ✅ |
| `/a200_0544/platform/odom/filtered` | `nav_msgs/Odometry` | 18119 | 约 44 Hz | `odom` → `base_link` | ⚠️ 未验证 |
| `/a200_0544/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 199 | 低频 | `map` | ⚠️ 未验证 |

- 格式：ROS 2 bag（mcap），rosbag2 元数据版本 9，ROS 2 Jazzy。
- 查看：`ros2 bag info <bag 目录>`。Mac / Windows 上不装 ROS 也能读：`pip install rosbags`。
- 注意：`rosbags` 0.9.x 不支持元数据版本 9。可以升级到 0.10 以上；或者像我们一样，另建一个文件夹放一份改成版本 8 的 `metadata.yaml` 副本（QoS 字段置空），用符号链接指向原始 `.mcap`。**原始 bag 没有任何改动。**

## 衍生数据

| 文件 | 生成方式 | 坐标系 | 说明 |
|---|---|---|---|
| `map/lab_map.*` | slam_toolbox（2026-09-29） | `map` | 分辨率 0.05 m；origin [-20.246, -26.585, 0] |
| `poses/*_poses.npy` | KISS-ICP（`deskew: true`，`min_range: 1.0`，`max_range: 50.0`） | 第一帧雷达坐标系 | 4109 × 4 × 4 齐次变换矩阵，与 bag 中点云逐帧一一对应 |
| `lab_map_3d.ply` | KISS-ICP 位姿 + `build_map.py`（体素 0.05 m） | 第一帧雷达坐标系 | 1,893,669 个点，22 MB |

### KISS-ICP 结果

| 指标 | 值 |
|---|---|
| 处理帧数 | 4109 / 4109 |
| 总路程 | 59.5 m |
| 回环误差（终点与起点距离） | 0.13 m（约 0.2%） |
| 处理速度 | 约 277 Hz（MacBook Air） |

## 需要注意

- 3D 地图和 KISS-ICP 位姿的坐标系是**第一帧雷达坐标系**，**不是** `map` 坐标系，两者尚未对齐。
- 已去掉离雷达 1 m 以内的点（主要是车身）。
- KISS-ICP 没有回环检测；本次路线回到起点，误差 0.13 m。
- 录制首尾各有一段静止时间，有人在附近走动时会留下少量动态物体的点。
- 已知问题：目前没有。

## 尚缺什么

- [ ] 3D 地图 / KISS-ICP 位姿与 `map` 和建筑模型的对齐
- [ ] 位姿精度的独立验证
- [ ] 需要重建端确认：bag 能否直接使用，位姿和标定需要什么格式
