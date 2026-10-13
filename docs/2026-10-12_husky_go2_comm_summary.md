# Husky A200 ↔ Unitree Go2 Communication — Progress Summary (2026-10-12)

> 中文版见下半部分。Detailed log, configs and full code: [`2026-10-12_husky_go2_comm_detailed.md`](2026-10-12_husky_go2_comm_detailed.md)

## Goal

Let the Clearpath Husky A200 (ROS 2 Jazzy) and the Unitree Go2 EDU (ROS 2 Foxy on its Jetson) exchange data and commands **wirelessly**, so the Husky can read the dog's state and send it motion commands.

## Architecture

```
 Go2 motion MCU ──eth0, DDS domain 0──▶ go2_bridge (unitree_sdk2) ──ROS 2 domain 1──▶ zenoh bridge (Go2)
 192.168.123.161                         on Go2 Jetson 192.168.123.18                        │
                                                                                            │ TCP 7447 over PSU-IOT Wi-Fi
                                                                                            ▼
                                                                    zenoh bridge (Husky) ──▶ Husky ROS 2 (Jazzy)
                                                                    10.170.33.10              /go2/odom, /go2/cmd_vel, /go2/command
```

- **`go2_bridge`** (new ROS 2 package, runs on the Go2's Jetson): reads the dog's state with `unitree_sdk2` and publishes `/go2/odom`; turns `/go2/cmd_vel` into `SportClient::Move()`; `/go2/command` (stand up / down / stop); 0.5 s cmd_vel watchdog.
- **`zenoh-bridge-ros2dds` v1.10.1** on both robots: carries only `/go2/*` over unicast TCP, so it works even though PSU-IOT blocks multicast and the two robots run different ROS 2 distros.

## What works ✅

| | Result |
|---|---|
| Network | Husky (`10.170.33.10`) and Go2 (`10.170.32.240`) on the same PSU-IOT /19 subnet; ping, SSH and TCP 7447 work both ways |
| SDK on Go2 | `unitree_sdk2` installed; standalone test reads `rt/sportmodestate` (position / yaw) |
| `go2_bridge` | Builds and runs on Foxy; `/go2/odom` published on the Go2 |
| Go2 → Husky | `ros2 topic echo /go2/odom` on the **Husky** receives the dog's odometry over Wi-Fi |
| Husky → Go2 | `/go2/cmd_vel` from the Husky reaches `go2_bridge` (Move called, watchdog StopMove logged) |

## Key problems solved

1. **SSH "failed"** – wrong IP typed (`10.172…` vs `10.170.32.240`).
2. **Multicast blocked on PSU-IOT** → ROS 2 auto-discovery can't cross robots → use zenoh (TCP unicast).
3. **Go2 has no internet** – NetworkManager gives `eth0` (internal net) the preferred default route → delete it.
4. **Go2 clock resets to 1970 / timezone is CST** → set time and `America/New_York` before apt/git.
5. **CycloneDDS header/library clash** between `unitree_sdk2` and ROS Foxy → CMake include order + RPATH.
6. **`bad_alloc` storm** – ROS (Fast DDS, domain 0) collides with the dog's own DDS traffic on domain 0 → run all ROS on the Go2 with `ROS_DOMAIN_ID=1`.
7. **ROS services don't cross zenoh** with Fast DDS (request < 20 bytes, dropped) → added the `/go2/command` topic.

## Open issues / next steps ⚠️

- [ ] Rebuild with the `/go2/command` version and test stand-up / walk / stand-down **on the real dog**; confirm the dog physically moves.
- [ ] `cmd_vel timeout` fired 3× while the Husky published continuously at 10 Hz → Wi-Fi delivery gaps > 0.5 s. Measure with `ros2 topic hz /go2/cmd_vel` on the Go2; consider `cmd_timeout` ≈ 1.0 s.
- [ ] Go2 zenoh bridge disconnected at 21:16:41 UTC ("Remote ROS 2 bridge left") – check whether Wi-Fi dropped again.
- [ ] Go2 Wi-Fi (`wlan0`) dropped once and needed a Jetson reboot → keep `power_save off`, watch stability.
- [ ] Make fixes permanent: `ipv4.never-default` on `Wired connection 1`; time sync (NTP) on the Go2.
- [ ] Startup is 4 manual terminals → write a launch script for the Go2 (and Husky).
- [ ] Verify sign/frame of `/go2/odom` twist; relate `go2/odom` to the Husky's TF tree for joint tasks.
- [ ] zenoh port 7447 is open on PSU-IOT without authentication – consider restricting.

---

# 中文版

## 目标

让 Clearpath Husky A200（ROS 2 Jazzy）和宇树 Go2 EDU（狗背 Jetson 上的 ROS 2 Foxy）**通过无线网络**互相通信：Husky 能读到狗的状态，并能给狗发运动指令。

## 实现方式

- **`go2_bridge`**（新写的 ROS 2 包，跑在 Go2 的 Jetson 上）：用 `unitree_sdk2` 读取狗的状态，发布 `/go2/odom`；订阅 `/go2/cmd_vel` 并调用 `SportClient::Move()`；`/go2/command` 话题负责站立 / 趴下 / 停止；0.5 秒收不到速度指令会自动停下。
- **两边都运行 `zenoh-bridge-ros2dds` v1.10.1**：只转发 `/go2/*`，走 TCP 单播，因此不受 PSU-IOT 屏蔽多播的影响，也能跨 ROS 版本（Foxy ↔ Jazzy）。

## 已完成 ✅

| | 结果 |
|---|---|
| 网络 | Husky（`10.170.33.10`）和 Go2（`10.170.32.240`）在同一个 PSU-IOT 子网，双向 ping、SSH、TCP 7447 都通 |
| Go2 上的 SDK | `unitree_sdk2` 安装完成，独立测试程序能读到狗的位置和朝向 |
| `go2_bridge` | 在 Foxy 上编译、运行成功，Go2 本机能收到 `/go2/odom` |
| Go2 → Husky | 在 **Husky** 上 `ros2 topic echo /go2/odom` 能收到狗的数据 |
| Husky → Go2 | Husky 发的 `/go2/cmd_vel` 已到达 `go2_bridge`（调用了 Move，日志中出现看门狗的 StopMove） |

## 解决的主要问题

1. **SSH 连不上**：IP 打错（`10.172…`，正确为 `10.170.32.240`）。
2. **PSU-IOT 屏蔽多播** → ROS 2 无法自动发现对方 → 改用 zenoh（TCP 单播）。
3. **Go2 上不了外网**：eth0（狗的内网）抢了默认路由 → 删除该路由。
4. **Go2 时间重置到 1970 年、时区为 CST** → 用 apt/git 前先改时间，并把时区设为 `America/New_York`。
5. **`unitree_sdk2` 与 ROS Foxy 的 CycloneDDS 头文件 / 库冲突** → 在 CMake 中调整 include 顺序并设置 RPATH。
6. **大量 `bad_alloc`**：ROS 的 Fast DDS 在域 0 与狗自己的 DDS 流量冲突 → Go2 上所有 ROS 程序使用 `ROS_DOMAIN_ID=1`。
7. **ROS 服务无法通过 zenoh**（Fast DDS 的请求不足 20 字节，被丢弃）→ 新增 `/go2/command` 话题替代。

## 尚未解决 / 下一步 ⚠️

- [ ] 编译带 `/go2/command` 的新版本，在真狗上测试站立、行走、趴下，确认狗确实动了。
- [ ] Husky 以 10 Hz 连续发布时，`cmd_vel timeout` 触发了 3 次 → Wi-Fi 上有超过 0.5 秒的传输间断。在 Go2 上用 `ros2 topic hz /go2/cmd_vel` 测量，考虑把 `cmd_timeout` 调到 1.0 秒左右。
- [ ] Go2 的 zenoh 桥在 21:16:41 UTC 断开（"Remote ROS 2 bridge left"），需要确认是不是 Wi-Fi 又掉线了。
- [ ] Go2 的 Wi-Fi（`wlan0`）掉线过一次，需要重启 Jetson 才恢复 → 保持 `power_save off`，观察稳定性。
- [ ] 把临时修复改成永久设置：`Wired connection 1` 设置 `ipv4.never-default`；Go2 时间同步（NTP）。
- [ ] 目前要手动开 4 个终端启动 → 为 Go2（和 Husky）编写启动脚本。
- [ ] 验证 `/go2/odom` 中速度的方向和坐标系；之后联合任务需要把 `go2/odom` 和 Husky 的 TF 关联起来。
- [ ] zenoh 的 7447 端口在 PSU-IOT 上无认证开放，考虑加以限制。
