# 样本：museum_1019

**中文** | [English](README.md)

## 概要

2026-10-19 在博物馆用 Husky A200 + Velodyne VLP-16 采集：入口大厅（`room01`），以及里面的展厅（走了两遍：`room04`、`room05`）。三段录像已合并成**同一坐标系下的一张点云地图**，另有 Nav2 栅格地图和两个 3D 模型（简洁的建筑模型 + 扫描网格）。**配准误差 7 cm RMS（room04 ↔ room05）、9 cm RMS（大厅 ↔ 展厅）；三段录像的地面高度相差 2 cm 以内。** 透过玻璃扫到的室外点、以及地面以下的反射点已删除。

![合并平面图：大厅（红）、room04（蓝）、room05（绿）](images/museum_floorplan.png)

## 采集信息

| 项目 | 内容 |
|---|---|
| 日期 / 时间 | 2026-10-19，09:58–10:43 EDT（13:58–14:43 UTC） |
| 地点 | ____（博物馆名称），入口大厅 + 内部展厅 |
| 机器人 | Clearpath Husky A200，序列号 a200-0544，ROS 命名空间 `a200_0544`，Ubuntu 24.04 + ROS 2 Jazzy |
| 雷达 | Velodyne VLP-16，10 Hz，外参 xyz = [-0.28, 0, 0.748] m，rpy = 0（与 `husky_1008` 相同） |
| 网络 | 博物馆没有网络；出发前机器人时钟已 NTP 同步 |
| 采集人 | 刘慧杭（Husky）。Go2 由 Yifei 单独采集，不在本样本中 |

| 录像 | 区域 | 时间（EDT） | 时长 | 路径 | 平均速度 | 帧数 | Bag 大小 |
|---|---|---|---|---|---|---|---|
| `room01` | 大厅 | 09:58:18–10:05:31 | 433 s | 79.2 m | 0.18 m/s | 4148 | 2.5 GiB |
| `room04` | 展厅，第 1 遍 | 10:33:33–10:37:15 | 222 s | 62.8 m | 0.28 m/s | 2052 | 1.2 GiB |
| `room05` | 展厅，第 2 遍 | 10:40:56–10:42:55 | 120 s | 37.7 m | 0.32 m/s | 1170 | 0.7 GiB |

另外两段录像没有放进来：`room02` 是不小心按到开始录制的；`room03` 录到一半机器人没电，bag 没有保存下来。路径长度和速度来自雷达里程计。

## 录制命令

```bash
ros2 bag record -o ~/dt_capture/Husky/<name>/bag \
  /a200_0544/sensors/lidar3d_0/points \
  /a200_0544/tf /a200_0544/tf_static \
  /a200_0544/platform/odom/filtered
```

录制器报告传输层丢失 164 / 164 / 21 条消息（0.35 % / 0.69 % / 0.16 %）。每段录像的 `ros2 bag info` 输出和元数据在 `recording_info/`。

## 目录结构

```
museum_1019/
├── museum_merged_clean.ply     # 合并点云，仅室内，5 cm，152 万点；字段 x y z intensity room（0 大厅，1 room04，2 room05）
├── clouds/room0{1,4,5}_aligned.ply   # 各段录像的点云（博物馆坐标系，4 cm）
├── poses/                      # 每帧雷达位姿，博物馆坐标系（.npy N×4×4 和 TUM .txt）
├── transforms.json             # 4×4 变换：各段 KISS-ICP 坐标系 -> 博物馆坐标系
├── map/museum_map.pgm + .yaml  # Nav2 栅格地图，0.05 m
├── model/
│   ├── museum_architectural.glb / .stl   # 地面、拉到天花板高度的墙、门过梁、展品方块、玻璃门
│   ├── museum_scan_model.glb             # Poisson 扫描网格，112 万三角面，平地面，玻璃门
│   ├── museum_scan_model.stl             # 同上，简化到 56 万三角面
│   ├── museum_scan_model_web.glb         # 32 万三角面
│   └── door.json                         # 实测的大厅 <-> room04 双开门
├── viewers/                    # 离线 HTML 查看器（任意浏览器打开，不需要网络）
├── images/                     # 平面图、室内外清理预览
├── recording_info/             # 每段录像的 bag_info / metadata / 话题
├── robot.yaml
└── scripts/                    # 处理流程代码（见下）
```

**原始 bag（4.4 GiB）不在本仓库**，和其他样本一样放到 OneDrive 的 `husky_data` 文件夹。

## 坐标系

`museum` 坐标系：地面 z = 0，墙与坐标轴对齐，原点 = `room04` 起点处的雷达位置（大厅 ↔ room04 门口），单位米。`transforms.json` 把每段录像的 KISS-ICP 坐标系（第一帧雷达位姿）变换到这个坐标系；`poses/` 里的位姿已经是这个坐标系。STL 文件单位是**米**（导入 CAD 时选米）。

## 处理流程

| 步骤 | 方法 | 计算时间（2 核） |
|---|---|---|
| 雷达里程计 | KISS-ICP，体素 0.2 m，范围 1–40 m，位姿约束为 SE(2)（地面是平的） | 14 分钟 |
| room05 → room04 | 在墙体栅格上做 2D 穷举相关搜索，再做平面约束的点到面 ICP | < 1 分钟 |
| 大厅 → 展厅 | 同样的搜索 + 自由空间一致性检查（墙不能落在另一张图看到是空的地方），再 ICP；通过开着的玻璃双开门连接 | 1.5 分钟 |
| 合并 / 清理 | 去掉跟着机器人走的人（大部分时间被激光穿过的格子）；从机器人路径向外漫水填充区分室内外，加上现场确认的两块室内区域 | 2 分钟 |
| 3D 模型 | 每个体素只用一段录像的点，Poisson 深度 10，Taubin 平滑，地面按激光穿过的空间铺平；建筑模型的墙拉到实测天花板高度；门按实测尺寸建模 | 3 分钟 |

`scripts/` 是实际运行的代码（路径是处理机器上的，需要改）：`map_room.py`（里程计）、`register.py`、`freespace.py`、`run_fs.py`（配准）、`build_final.py`（合并、Nav2 地图）、`interior.py`（室内范围）、`prep_mesh_cloud.py`、`poisson.py`、`floor.py`、`build_model.py`（扫描网格）、`arch_model.py`、`export_arch.py`、`door_model.py`（建筑模型、门）。

## 结果

| 检查项 | 数值 |
|---|---|
| room04 ↔ room05 | 73 % 的墙面点在 15 cm 以内，7 cm RMS |
| 大厅 ↔ 展厅 | 9 cm RMS |
| 各段录像地面高度 | 相差 2 cm 以内 |
| 单段录像墙面噪声 | 3.2 cm RMS |
| 天花板高度 | 2.6–3.4 m |
| 建模面积 | 占地约 660 m²，地面 556 m² |
| 大厅 ↔ room04 门 | 门洞 2.25 × 2.13 m，两扇 1.12 m 门板，墙厚约 0.9 m，录制时向大厅一侧打开约 90° |

## 注意事项

- 轮式里程计**不能**用来连接各段录像：同一次开机内，`room04` 和 `room05` 之间航向漂了约 38°。
- 各段录像自身的漂移，使 `room04` 和 `room05` 的部分墙面相差 5–10 cm。
- 大厅北侧（靠近玻璃门）扫描很少，那里的几何比较粗糙。
- 玻璃（门、窗）扫不到，在模型里是缺口；只有大厅 ↔ room04 的门单独建了模。
- VLP-16 只能看到机器人周围约 3.4 m 内的地面；模型地面是按激光穿过的空间补齐的。

## 待完成

- [ ] 把原始 bag 上传到 OneDrive
- [ ] 博物馆坐标系与建筑模型对齐
- [ ] 同一次采集的 Go2 数据（Yifei）
