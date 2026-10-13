# Husky A200 操作手册（ROS 2 Jazzy）

[English](README.md) | **中文**

Clearpath Husky A200（PSU13，序列号 `a200-0544`）在 **Ubuntu 24.04 + ROS 2 Jazzy** 下的配置、日常操作流程和排错记录。

> 旧系统是 ROS Kinetic，2026 年 8～9 月按 Clearpath 官方文档重装为 Jazzy。
> 项目概览、3D 地图结果和 Go2 通信说明见 [README.md](README.md)。

**目录**：[硬件与网络](#硬件与网络) · [开机与连接](#1-开机) · [遥控](#5-遥控) · [SLAM 建图](#6-slam-建图) · [定位与导航](#7-定位与自动导航nav2) · [数据采集](#8-数据采集) · [3D 点云地图](#9-生成-3d-点云地图mac不需要-ros) · [系统架构要点](#系统架构要点) · [排错记录](#排错记录)

---

## 硬件与网络

| 项目 | 值 |
|---|---|
| 序列号 | `a200-0544` |
| ROS 命名空间 | `a200_0544`（注意是下划线） |
| 机器人用户名 | `admin` |
| 机器人有线 IP（br0） | `192.168.131.1` |
| 无线路由器（Waypoint） | `192.168.131.51`，SSID：`PSU13 Waypoint` |
| Velodyne VLP-16 | `192.168.131.20`，UDP 端口 2368，约 10 Hz |
| 雷达外参（`base_link` → `lidar3d_0_link`） | `xyz = [-0.28, 0.0, 0.748]` m，`rpy = [0, 0, 0]`（2026-10-08 实测，约 ±1 cm） |
| 手柄 | Logitech F710（robot.yaml 中 `controller: logitech`） |
| 其他传感器（尚未配置） | UM7 IMU、NovAtel GPS |

> 密码不要写进本仓库。

配置文件：
- `samples/husky_1008/robot.yaml`：机器人上 `/etc/clearpath/robot.yaml` 的快照（2026-10-08 录数据时的版本，含实测外参）
- `config/50-clearpath-bridge.yaml`：机器人上的 `/etc/netplan/50-clearpath-bridge.yaml`（br0 网桥，桥接 `enp3s0` + `eno1`）

---

## 日常操作流程

### 1. 开机
1. 放在空旷处，确保有人能随时按急停。
2. 按电源键，旋开急停，确认 lockout 钥匙没锁。
3. 等 1～2 分钟让 Clearpath 服务启动。

### 2. 连接
Mac 连 Wi-Fi `PSU13 Waypoint`，然后：

```bash
ssh admin@192.168.131.1
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash
```

建议所有长时间运行的程序都放在 tmux 里，Wi-Fi 断了也不会中断：

```bash
tmux new -s husky        # 断线后重新 SSH，再运行：tmux attach -t husky
```

> tmux 里每个新窗口都要重新 `source` 上面两个 setup 文件（除非已经写进 `~/.bashrc`）。

### 3. 检查状态

```bash
systemctl list-units | grep clearpath              # platform / sensors 服务应为 running
ros2 topic list | grep a200_0544
ros2 topic hz /a200_0544/sensors/lidar3d_0/points  # 应约 10 Hz
```

### 4. 可视化（Mac 上用 Foxglove）
- 用 **Foxglove 桌面版**（网页版连不上 `ws://` 局域网地址）。
- Open connection → Foxglove WebSocket → `ws://192.168.131.1:8765`
- 3D 面板中点亮 `/a200_0544/sensors/lidar3d_0/points` 的眼睛图标，Display frame 选 `base_link`。

### 5. 遥控
- **手柄（Logitech F710）**：按住 **LB** + 推左摇杆。手柄优先级最高，导航时按住 LB 可以随时接管。
- **键盘**（在 SSH 终端里）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -r cmd_vel:=/a200_0544/cmd_vel
```

- **命令行测试**（轮子先架空）：

```bash
ros2 topic pub -r 10 /a200_0544/cmd_vel geometry_msgs/msg/TwistStamped \
"{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

### 6. SLAM 建图

```bash
# 只需安装一次
sudo apt install ros-jazzy-clearpath-nav2-demos tmux

ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:=/etc/clearpath/ \
  scan_topic:=/a200_0544/sensors/lidar3d_0/scan
```

- 真机**不要**加 `use_sim_time:=true`。
- Foxglove 中点亮 `/a200_0544/map`，Display frame 改为 `map`。
- 慢速行驶（约 0.3 m/s），慢转弯，沿墙走，最后回到起点闭环。

保存地图（另开一个 tmux 窗口，**先别关 SLAM**）：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map \
  --ros-args -r map:=/a200_0544/map
```

会生成 `lab_map.pgm` 和 `lab_map.yaml`。仓库里的示例地图在 `samples/husky_1008/map/`。

### 7. 定位与自动导航（Nav2）

先启动定位，给初始位置，再启动导航。地图路径用绝对路径，不要用 `~`。

```bash
# 窗口 0：在已有地图上定位（AMCL）
ros2 launch clearpath_nav2_demos localization.launch.py \
  setup_path:=/etc/clearpath/ \
  map:=/home/admin/maps/lab_map.yaml \
  scan_topic:=/a200_0544/sensors/lidar3d_0/scan

# 窗口 1：导航（等 localization 输出 "Managed nodes are active" 后再启动）
ros2 launch clearpath_nav2_demos nav2.launch.py \
  setup_path:=/etc/clearpath/ \
  scan_topic:=/a200_0544/sensors/lidar3d_0/scan
```

在 Foxglove 中设置初始位置和目标点：
1. Display frame 选 `map`，点亮 `/a200_0544/map`、点云和 `/a200_0544/plan`（规划路径）。
2. 3D 面板设置 → Publish：**Pose estimate** 话题设为 `/a200_0544/initialpose`，**Pose** 话题设为 `/a200_0544/goal_pose`。
3. 工具栏里带小三角的圆点图标是发布工具（最上面的箭头是选择工具）。点小三角选择类型。
4. 选 Pose estimate，在小车实际位置按住，朝车头方向拖动后松开。
5. 用手柄慢慢原地转一圈、前后开一两米，等激光点贴到地图的墙上。
6. 选 Pose，在前方 1～2 米的空地拖出目标点，小车会自动开过去。

如果 Foxglove 里还没有地图，可以用命令行发初始位置（小车停在建图起点时）：

```bash
ros2 topic pub --once /a200_0544/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

第一次测试时把速度限低一点，旁边留人拿着手柄。

### 8. 数据采集

```bash
df -h ~              # 先确认剩余空间
mkdir -p ~/bags
ros2 bag record -o ~/bags/husky_<日期> \
  /a200_0544/sensors/lidar3d_0/points \
  /a200_0544/tf /a200_0544/tf_static \
  /a200_0544/platform/odom/filtered \
  /a200_0544/amcl_pose
ros2 bag info ~/bags/husky_<日期>    # 检查消息数量
```

- 电池续航约半小时，单次录制控制在几分钟以内。
- `amcl_pose` 只有 localization 运行时才有。

拷到 Mac：

```bash
scp -r admin@192.168.131.1:~/bags/husky_<日期> ~/Desktop/
```

### 9. 生成 3D 点云地图（Mac，不需要 ROS）

```bash
python3 -m venv ~/kiss-env && source ~/kiss-env/bin/activate
pip install "kiss-icp[all]" rosbags numpy
kiss_icp_pipeline --config samples/husky_1008/kiss.yaml --visualize \
  --topic /a200_0544/sensors/lidar3d_0/points husky_<日期>
python samples/husky_1008/build_map.py husky_<日期> <results 中的 *_poses.npy>
```

- `kiss.yaml` 中 `min_range: 1.0` 用来去掉车身上的点。
- 输出的 `lab_map_3d.ply` 用 CloudCompare 打开。
- `rosbags` 0.9.x 不支持 metadata version 9，处理方法见 [samples/husky_1008/README.md](samples/husky_1008/README.md)。

---

## 系统架构要点

- Clearpath Jazzy 的所有配置都来自 `/etc/clearpath/robot.yaml`，改完后运行：
  ```bash
  sudo systemctl restart clearpath-robot.service
  ```
- 传感器写进 robot.yaml 的 `sensors:` 段即可，驱动由 `clearpath-sensors.service` 自动启动。
- 速度指令流向：`joy_teleop` / `twist_marker_server` / `cmd_vel` → **twist_mux** → `platform/cmd_vel` → 轮子。手柄优先级最高，按住使能键时会覆盖程序发出的指令。
- Jazzy 下速度消息类型为 **`TwistStamped`**，不是 `Twist`。
- TF 话题带命名空间：`/a200_0544/tf`、`/a200_0544/tf_static`。
- 机器人是 Ubuntu Server，没有图形界面；可视化在 Mac（Foxglove）或 Ubuntu 桌面电脑（RViz）上做。
- 查看服务状态时加 `--no-pager`（如 `systemctl status clearpath-robot.service --no-pager`），否则会进入分页器。

### 在 Ubuntu 桌面电脑上用 RViz

```bash
sudo apt install ros-jazzy-clearpath-desktop   # 需先添加 Clearpath apt 源
mkdir -p ~/clearpath
scp admin@192.168.131.1:/etc/clearpath/robot.yaml ~/clearpath/
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_robot.launch.py namespace:=a200_0544
```

不装 Clearpath 包时，也可以直接用 rviz2：

```bash
rviz2 --ros-args -r /tf:=/a200_0544/tf -r /tf_static:=/a200_0544/tf_static
```

PointCloud2 不显示时，把 Reliability Policy 改为 **Best Effort**。

---

## 排错记录

| 问题 | 原因 | 解决 |
|---|---|---|
| `Unknown topic '/a200-0544/cmd_vel'` | 命名空间用的是下划线，不是短横线 | 用 `/a200_0544/...` |
| Mac `ssh` 超时，ping 不通 `192.168.131.1` | 重装后有线网口没有 IP，没有 br0 网桥 | 新建 `/etc/netplan/50-clearpath-bridge.yaml`（见 `config/`），然后 `sudo netplan try` |
| netplan 报 `unknown key 'br0'` | `bridges:` 缩进错了，被放到了 `ethernets:` 下面 | `bridges:` 与 `ethernets:` 对齐（都缩进 2 个空格）；用 heredoc 写文件比 nano 更不容易出错 |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | 重装后 SSH 主机密钥变了 | Mac 上运行 `ssh-keygen -R 192.168.131.1` |
| 用户名 `administrator` 登录失败 | 旧系统用户名是 administrator，新系统是 `admin` | `ssh admin@192.168.131.1` |
| `ssh.service` 显示 inactive | Ubuntu 24.04 由 `ssh.socket` 按需启动 SSH，属正常现象 | 无需处理 |
| `ros2 topic hz .../points` 没有输出 | 话题存在但雷达没发数据（雷达没通电或网线没接） | 依次检查 `ping 192.168.131.20`、`sudo tcpdump -i br0 -c 5 udp port 2368`，确认雷达在转、接线盒有电 |
| `package 'clearpath_viz' not found` | 机器人上没有这个包，或桌面电脑没装 | 在桌面电脑上安装 `ros-jazzy-clearpath-desktop` |
| Foxglove 能连上但看不到点云 | 话题默认是隐藏的 | 点亮话题右边的眼睛图标 |
| Nav2 报 `"map" passed to lookupTransform ... does not exist` | 定位还没起来，或还没给初始位置 | 先启动 localization，等 `Managed nodes are active`，发 initialpose 后再启动 nav2；用 `ros2 node list` 确认没有重复的定位进程 |

---

## 待办

- [x] 排查雷达没有数据的问题
- [x] 实测雷达相对于 base_link 的位置，更新 robot.yaml 的 `xyz`
- [x] 完成第一张 SLAM 地图（`samples/husky_1008/map/`）
- [x] 用 Nav2 做基于地图的自动导航
- [x] 录制第一份雷达数据并生成 3D 点云地图
- [ ] 在 robot.yaml 中加入 UM7 IMU 和 NovAtel GPS
- [ ] 把 3D 地图 / KISS-ICP 位姿和 `map` 坐标系、建筑模型对齐
- [ ] Husky ↔ Go2：在真狗上测试 `/go2/command`，写启动脚本（见 [docs/](docs/)）
