# HuskyA200

Clearpath Husky A200（PSU13）在 **Ubuntu 24.04 + ROS 2 Jazzy** 下的配置、操作流程和排错记录。

> 旧系统是 ROS Kinetic。2026 年 8～9 月按 Clearpath 官方文档重装为 Jazzy。

---

## 硬件与网络

| 项目 | 值 |
|---|---|
| 序列号 | `a200-0544` |
| ROS 命名空间 | `a200_0544`（注意是下划线） |
| 机器人用户名 | `admin` |
| 机器人有线 IP（br0） | `192.168.131.1` |
| 无线路由器（Waypoint） | `192.168.131.51`，SSID：`PSU13 Waypoint` |
| Velodyne VLP-16 | `192.168.131.20`，UDP 端口 2368 |
| 其他传感器（尚未配置） | UM7 IMU、NovAtel GPS |
| 手柄 | PS4（robot.yaml 中 `controller: ps4`） |

> 密码不要写进本仓库。

配置文件备份见 `config/`：
- `robot.yaml` → 机器人上的 `/etc/clearpath/robot.yaml`
- `50-clearpath-bridge.yaml` → 机器人上的 `/etc/netplan/50-clearpath-bridge.yaml`

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
- **手柄**：按住 L1 + 推左摇杆。
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

# 用 tmux 防止断线中断程序
tmux new -s husky        # 断线后：tmux attach -t husky

ros2 launch clearpath_nav2_demos slam.launch.py \
  setup_path:=/etc/clearpath/ \
  scan_topic:=/a200_0544/sensors/lidar3d_0/scan
```

- Foxglove 中点亮 `/a200_0544/map`，Display frame 改为 `map`。
- 慢速行驶（约 0.3 m/s），慢转弯，沿墙走，最后回到起点闭环。

保存地图：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map \
  --ros-args -r map:=/a200_0544/map
```

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
- 机器人是 Ubuntu Server，无图形界面；可视化在 Mac（Foxglove）或 Ubuntu 桌面电脑（RViz）上做。

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
| `Unknown topic '/a200-0544/cmd_vel'` | 命名空间用下划线，不是短横线 | 用 `/a200_0544/...` |
| Mac `ssh` 超时，ping 不通 `192.168.131.1` | 重装后有线网口没有 IP，没有 br0 网桥 | 新建 `/etc/netplan/50-clearpath-bridge.yaml`（见 `config/`），然后 `sudo netplan try` |
| netplan 报 `unknown key 'br0'` | `bridges:` 缩进错了，被放在 `ethernets:` 下面 | `bridges:` 与 `ethernets:` 对齐（都缩进 2 个空格） |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | 重装后 SSH 主机密钥变了 | Mac 上运行 `ssh-keygen -R 192.168.131.1` |
| 用户名 `administrator` 登录失败 | 旧系统用户名是 administrator，新系统是 `admin` | `ssh admin@192.168.131.1` |
| `ssh.service` 显示 inactive | Ubuntu 24.04 由 `ssh.socket` 按需启动 SSH，属正常现象 | 无需处理 |
| `package 'clearpath_viz' not found` | 机器人上没有这个包，或桌面电脑没装 | 在桌面电脑上安装 `ros-jazzy-clearpath-desktop` |
| Foxglove 能连上但看不到点云 | 话题默认是隐藏的 | 点亮话题右边的眼睛图标 |

---

## 待办

- [ ] 排查 `ros2 topic hz .../points` 没有输出的问题（依次检查 ping、tcpdump、sensors 日志）
- [ ] 测量雷达相对于 base_link 的真实位置，更新 robot.yaml 的 `xyz`（目前是估算值 `[0, 0, 0.5]`）
- [ ] 在 robot.yaml 中加入 UM7 IMU 和 NovAtel GPS
- [ ] 完成第一张 SLAM 地图并存入 `maps/`
- [ ] 用 Nav2 做基于地图的自动导航
