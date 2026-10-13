# Husky A200 ↔ Unitree Go2 Communication — Detailed Log (2026-10-12, updated 2026-10-13)

Companion to the short summary: [`2026-10-12_husky_go2_comm_summary.md`](2026-10-12_husky_go2_comm_summary.md).
This file records the full setup, every problem hit on 2026-10-12 with its root cause and fix, the exact commands, and the full source code as of the end of the day.

> **Status (2026-10-13):** Go2 → Husky odometry over Wi-Fi works. Husky → Go2 `cmd_vel` reaches `go2_bridge`. **Husky → Go2 `/go2/command` works: `stand_up` and `stand_down` sent from the Husky make the real dog stand up and lie down** (§4.11). Walking via `cmd_vel` while standing is the next test.

---

## 1. System configuration

### 1.1 Clearpath Husky A200 (`a200-0544`)

| Item | Value |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | ROS 2 **Jazzy** (Clearpath stack, `source /etc/clearpath/setup.bash`) |
| ROS namespace | `/a200_0544` (underscore) |
| Husky drive topic | `/a200_0544/cmd_vel` — `geometry_msgs/TwistStamped` |
| RMW | Clearpath default; Fast DDS inferred from the zenoh service-request format (see §4.9) |
| Login | `ssh admin@192.168.131.1` (user **admin**, not administrator) |
| Wired network | `br0` = `192.168.131.1/24`, bridging `enp3s0` + `eno1` (`/etc/netplan/50-clearpath-bridge.yaml`) |
| Robot Wi-Fi router | "waypoint" router `192.168.131.51` on the Husky's 192.168.131.0/24 network; the Mac joins this Wi-Fi to SSH in |
| Campus Wi-Fi | `wlp2s0` = **`10.170.33.10/19`** (same /19 as the Go2 on PSU-IOT) |
| zenoh | `zenoh-bridge-ros2dds` **1.10.1** (amd64, Eclipse apt repo) |
| zenoh config | `~/go2_zenoh.json5` (same content as `go2_bridge/config/go2_zenoh.json5`) |

### 1.2 Unitree Go2 EDU (Jetson expansion PC)

| Item | Value |
|---|---|
| OS | Ubuntu **20.04.5** LTS, kernel `5.10.104-tegra`, **aarch64** |
| RAM / swap | 15 GiB / 7.5 GiB |
| ROS | ROS 2 **Foxy** and ROS 1 Noetic both installed. `~/.bashrc` asks `ros:foxy(1) noetic(2) ?` at every login → **answer `1`** |
| Login | `ssh unitree@10.170.32.240` (Wi-Fi) or `ssh unitree@192.168.123.18` (wired) |
| Internal network | `eth0` = `192.168.123.18/24`; motion MCU = `192.168.123.161`; NetworkManager connection `Wired connection 1` |
| Campus Wi-Fi | `wlan0` = **`10.170.32.240/19`** on PSU-IOT, gateway `10.170.32.1`, NetworkManager connection `go2-psu-iot` (credentials from CoE IT) |
| Unitree DDS | The dog's own services publish `rt/*` topics with Unitree's CycloneDDS on **domain 0** over `eth0` |
| unitree_sdk2 | Cloned from GitHub `main` on 2026-10-12, installed to `/opt/unitree_robotics` (`libunitree_sdk2.a`, bundled `libddsc.so.0`, `libddscxx.so.0`) |
| ROS domain for our nodes | **`ROS_DOMAIN_ID=1`** (domain 0 is the dog's internal traffic, see §4.8) |
| RMW for our nodes | `rmw_fastrtps_cpp` (Cyclone-based RMW would load a second libddsc next to the SDK's) |
| zenoh | `zenoh-bridge-ros2dds` **1.10.1** (arm64, Eclipse apt repo) |
| Workspace | `~/go2_ws/src/HuskyA200` (this repo) → package `go2_bridge` |
| Clock | Resets to 1969/1970 on every boot; factory timezone was Asia/Shanghai (CST) → changed to `America/New_York` |
| apt mirrors | Unitree default (Tsinghua / USTC, China) — slow / partly failing from the US |

### 1.3 Other machines

| Machine | Address | Role |
|---|---|---|
| Lab desktop `E5-AE-ROS-PC` | `wlp131s0f0` = `10.170.32.181/19` | SSH to the Go2 over Wi-Fi |
| Labmate's Mac | wired to Go2, `192.168.123.222` | SSH over the dog's internal network (survives Wi-Fi drops) |
| Huihang's Mac | `192.168.131.x` on the waypoint Wi-Fi | SSH to the Husky; pushes code to GitHub |
| Huihang's Mac (10-13) | additionally **wired to the Go2**, manual `192.168.123.x/24`, router field empty | `ssh unitree@192.168.123.18` — survives Go2 Wi-Fi drops |

### 1.4 Network topology

```
                     PSU-IOT Wi-Fi  10.170.32.0/19  (multicast blocked, unicast OK)
        ┌───────────────────────────────┬───────────────────────────────┐
        │                               │                               │
  Husky wlp2s0                    Go2 Jetson wlan0                Lab desktop
  10.170.33.10                    10.170.32.240                   10.170.32.181
        │                               │
  Husky br0 192.168.131.1         Go2 Jetson eth0 192.168.123.18
        │                               │
  waypoint router 192.168.131.51  Go2 motion MCU 192.168.123.161
        │                         (+ labmate's Mac 192.168.123.222 when wired)
  Mac (SSH to Husky)
```

---

## 2. Software architecture

```
 Go2 motion MCU ──eth0, DDS domain 0 (rt/sportmodestate, sport RPC)──▶ go2_bridge  [Go2 Jetson, Foxy, ROS_DOMAIN_ID=1, Fast DDS]
                                                                        │  pub /go2/odom      (nav_msgs/Odometry, 50 Hz)
                                                                        │  sub /go2/cmd_vel   (geometry_msgs/Twist)
                                                                        │  sub /go2/command   (std_msgs/String)
                                                                        ▼
                                                     zenoh-bridge-ros2dds (Go2)  -e tcp/10.170.33.10:7447
                                                                        │   only /go2/* allowed
                                                                        ▼   TCP over PSU-IOT
                                                     zenoh-bridge-ros2dds (Husky) -l tcp/0.0.0.0:7447
                                                                        │
                                                                        ▼
                                                     Husky ROS 2 Jazzy nodes / CLI
```

### Interface of `go2_bridge`

| Name | Type | Direction | Meaning |
|---|---|---|---|
| `/go2/odom` | `nav_msgs/Odometry` | out | Pose from `rt/sportmodestate` (position, IMU rpy → quaternion), twist rotated into body frame. Frames `go2/odom` → `go2/base_link` |
| `/go2/cmd_vel` | `geometry_msgs/Twist` | in | `SportClient::Move(vx, vy, wz)`, clamped by `max_vx/max_vy/max_wz`. Watchdog: `StopMove()` if no message for `cmd_timeout` (0.5 s) |
| `/go2/command` | `std_msgs/String` | in | `stand_up` (StandUp → BalanceStand), `stand_down`, `recovery_stand`, `stop` — `stand_up` / `stand_down` **verified from the Husky 2026-10-13** |
| `/go2/stand_up`, `/go2/stand_down`, `/go2/stop` | `std_srvs/Trigger` | service | Work locally on the Go2 only; **do not cross zenoh** (§4.9) |

Parameters: `network_interface` (required, `eth0`), `max_vx` 0.5, `max_vy` 0.3, `max_wz` 0.8, `cmd_timeout` 0.5, `odom_frame`, `base_frame`, `odom_rate` 50.

### Repository layout (added today)

```
HuskyA200/
├── go2_bridge/                 ROS 2 package (built on the Go2)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/go2_zenoh.json5  zenoh allow-list, used on both robots
│   └── src/go2_bridge.cpp
├── go2_tools/                  not built by colcon (COLCON_IGNORE)
│   ├── COLCON_IGNORE
│   └── go2_state_test/         SDK-only read test, no ROS
└── docs/                       these two notes
```

---

## 3. Procedures

### 3.1 One-time setup on the Go2

```bash
# login, answer 1 (foxy)
# 1) time + timezone (clock resets on boot)
sudo timedatectl set-timezone America/New_York
sudo date -s "YYYY-MM-DD HH:MM:SS"
# 2) internet: drop the default route that points into the dog's internal net
sudo ip route del default via 192.168.123.1 dev eth0
sudo nmcli con mod "Wired connection 1" ipv4.never-default yes   # permanent (recommended; not yet confirmed applied)
sudo iw dev wlan0 set power_save off                             # recommended after the Wi-Fi drop
# 3) unitree_sdk2 (do NOT build the examples: they are many and slow on the Jetson)
sudo apt install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev unzip
git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cd ~/unitree_sdk2 && mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics -DBUILD_EXAMPLES=OFF
make -j2 && sudo make install
# 4) go2_bridge
mkdir -p ~/go2_ws/src && cd ~/go2_ws/src
git clone https://github.com/greenaandblue/HuskyA200.git
cd ~/go2_ws && source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_bridge --parallel-workers 1
ldd ~/go2_ws/install/go2_bridge/lib/go2_bridge/go2_bridge | grep dds   # must point to /opt/unitree_robotics/lib
# 5) zenoh (same version as the Husky)
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install -y zenoh-bridge-ros2dds=1.10.1
```

### 3.2 One-time setup on the Husky

```bash
ssh admin@192.168.131.1
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update && sudo apt install -y zenoh-bridge-ros2dds      # installed 1.10.1
# copy go2_bridge/config/go2_zenoh.json5 to ~/go2_zenoh.json5
```

### 3.3 Updating the code on the Go2

Edit on the Mac → `git push` → on the Go2:

```bash
cd ~/go2_ws/src/HuskyA200 && git pull
cd ~/go2_ws && rm -rf build/go2_bridge install/go2_bridge
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2_bridge --parallel-workers 1
```

### 3.4 Daily startup (4 terminals, order A → C → B → D)

**A — Go2: go2_bridge** (keep running)
```bash
source /opt/ros/foxy/setup.bash
source ~/go2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=1
unset CYCLONEDDS_URI
ros2 run go2_bridge go2_bridge --ros-args -r __ns:=/go2 -p network_interface:=eth0 -p max_vx:=0.2
# expect: [go2.go2_bridge]: go2_bridge up on interface 'eth0'
```

**C — Husky: zenoh (listen)** (keep running)
```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash
zenoh-bridge-ros2dds -c ~/go2_zenoh.json5 -l tcp/0.0.0.0:7447
```

**B — Go2: zenoh (connect)** (keep running — easy to forget; without it the Husky's zenoh log shows no `Remote bridge … announces` lines and `ros2 topic pub -w 1` waits forever)
```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
zenoh-bridge-ros2dds -c ~/go2_ws/src/HuskyA200/go2_bridge/config/go2_zenoh.json5 -e tcp/10.170.33.10:7447
```

**D — Husky: test**
```bash
source /opt/ros/jazzy/setup.bash
source /etc/clearpath/setup.bash
ros2 topic list | grep go2
ros2 topic echo /go2/odom --once            # works (verified)
ros2 topic hz /go2/odom                     # expect ~50 Hz
# motion (clear area first) — stand_up / stand_down verified 2026-10-13:
ros2 topic pub --once -w 1 /go2/command std_msgs/msg/String "{data: stand_up}"
ros2 topic pub -r 10 /go2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"   # Ctrl+C → dog stops after 0.5 s
ros2 topic pub --once -w 1 /go2/command std_msgs/msg/String "{data: stand_down}"
```

Notes: Foxy's `ros2 topic echo` has no `--once` (Jazzy does). Use Tab completion for long paths — several of today's errors were typos.

---

## 4. Chronological log: problems, root causes, fixes

### 4.1 Desktop could not SSH to the Go2
- **Symptom:** `ssh unitree@10.172.32.240` → `Connection timed out`, while the labmate's Mac (wired) could SSH.
- **Cause:** The Mac used the dog's internal wired network (`192.168.123.18`), unrelated to PSU-IOT; the desktop used a mistyped IP (`10.172` instead of `10.170`).
- **Fix:** `ssh unitree@10.170.32.240` works. Husky → Go2 `ping` (2–5 ms) and `nc -zv 10.170.32.240 22` succeed; Go2 → Husky `nc 10.170.33.10 7447` also succeeds.

### 4.2 Multicast does not cross PSU-IOT
- **Test:** `ros2 multicast send` on the Go2, `ros2 multicast receive` on the Husky → nothing received.
- **Consequence:** default ROS 2 DDS discovery cannot connect the two robots.
- **Decision:** `zenoh-bridge-ros2dds` over TCP unicast (also bridges Foxy ↔ Jazzy for standard message types and lets us allow-list `/go2/*` only).

### 4.3 Go2 login prompt
- `ros:foxy(1) noetic(2) ?` — choosing `2` loads ROS 1 (`ros2: command not found`); sourcing Foxy afterwards mixes environments. Always choose **1**.

### 4.4 Go2 clock and timezone
- Clock was `Thu Jan 1 1970`, timezone CST (Asia/Shanghai). Fixed with `timedatectl set-timezone America/New_York` + `date -s`. The clock resets again after reboot (seen as "Last login 1969").

### 4.5 Go2 has no internet
- **Symptom:** `ping github.com` → `Temporary failure in name resolution`; `ping 8.8.8.8` fails.
- **Cause:** `ip route` showed `default via 192.168.123.1 dev eth0 metric 20100` ahead of `default via 10.170.32.1 dev wlan0 metric 20600`; `192.168.123.1` does not exist.
- **Fix:** `sudo ip route del default via 192.168.123.1 dev eth0` (temporary; returns on reboot). Permanent: `nmcli con mod "Wired connection 1" ipv4.never-default yes`. Does **not** affect Husky ↔ Go2 traffic (same subnet, direct route).

### 4.6 SDK build "froze", SSH to the Go2 lost
- **Symptom:** `make -j4` of `unitree_sdk2` stopped at 45 % (linking G1/H1 examples), Ctrl+C had no effect, new SSH → `No route to host`, but the labmate's wired SSH still worked.
- **Diagnosis:** `free -h` showed 12 GiB free, no swap → not out of memory. `nmcli dev status`: `wlan0 disconnected`; reconnecting failed with "The Wi-Fi network could not be found". The **Wi-Fi interface dropped**, killing the Wi-Fi SSH session.
- **Fix:** Jetson reboot (`sudo reboot`, dog lying down) restored Wi-Fi. Rebuilt the SDK with `-DBUILD_EXAMPLES=OFF` / `make -j2`. Recommended `iw dev wlan0 set power_save off`; do heavy work over the wired SSH.

### 4.7 `go2_bridge` compile errors
1. **CycloneDDS header clash** — `ddsi_sertype` / `dds_create_topic_sertype` not declared. `unitree_sdk2` ships CycloneDDS 0.10 C++ headers, but `/opt/ros/foxy/include` (older CycloneDDS C headers) was searched first. **Fix:** `target_include_directories(... BEFORE ...)` for the SDK include dirs, plus `BUILD_RPATH/INSTALL_RPATH=/opt/unitree_robotics/lib` with `-Wl,--disable-new-dtags` so the SDK's `libddsc` also wins at runtime over `LD_LIBRARY_PATH`. Verified with `ldd`.
2. **Foxy service callbacks** — `AnyServiceCallback::set` failed: Foxy's rclcpp cannot deduce generic (`auto`) lambdas. **Fix:** explicit `std::shared_ptr<Trigger::Request/Response>` parameter types.

### 4.8 `bad_alloc caught: std::bad_alloc` flood
- **Symptom:** `go2_bridge` printed `bad_alloc` repeatedly and exited before its "up" log; **`ros2 topic echo /go2/odom` (no SDK involved) did the same**.
- **Isolation:** `go2_tools/go2_state_test` (SDK only, no ROS, `env -u CYCLONEDDS_URI`) read `rt/sportmodestate` correctly → the SDK and firmware match.
- **Root cause:** ROS 2 nodes (Fast DDS) on default **domain 0** discover the dog's own Unitree CycloneDDS participants on the same domain and fail on their discovery data.
- **Fix:** `export ROS_DOMAIN_ID=1` for every ROS process on the Go2 (bridge, CLI, zenoh). `unset CYCLONEDDS_URI` is also used (the login environment points it at a config for ROS's older CycloneDDS). Result: `go2_bridge up on interface 'eth0'`, `/go2/odom` prints locally.
- Typos met on the way: `ROS_DOMAIN=1` (must be `ROS_DOMAIN_ID`), `__ns:/go2` (must be `__ns:=/go2`).

### 4.9 zenoh link and ROS services
- zenoh repo URL typo `eclilpse` → corrected to `eclipse`; config path typo `go2_wa` → `go2_ws`. `systemctl not found` warning on install is harmless (bridge is started by hand).
- **Topics work:** Husky `ros2 topic echo /go2/odom` received the dog's odometry; zenoh logs show `/go2/odom` subscriber and `/go2/cmd_vel` publisher routes ("Allowed").
- **Services fail:** Husky zenoh log:
  `Route Service Client (ROS:/go2/stand_up -> Zenoh:go2/stand_up): received invalid request: [00, 01, 00, 00, 00, 00, 00, 00] (less than 20 bytes) dropping it`.
  The bridge expects the request id inside the payload (CycloneDDS layout: 4-byte CDR header + 16-byte id); Fast DDS carries it outside the payload. The Go2 side must stay on Fast DDS (§1.2), so services cannot be bridged.
- **Fix (untested):** added the `/go2/command` (`std_msgs/String`) topic for stand_up / stand_down / recovery_stand / stop. Services are kept for local use on the Go2.

### 4.10 First motion test (old binary, before `/go2/command`)
- Husky published `/go2/cmd_vel` `{linear.x: 0.1}` at 10 Hz for ~8 s (21:15:30–21:15:38 UTC, 78 messages).
- `go2_bridge` (started 21:01:17 UTC) logged `cmd_vel timeout -> StopMove` at 21:15:35, :36 and :38 — so the messages **did arrive and `Move()` was called**, but with gaps > 0.5 s while the publisher was still running.
- 21:16:41 UTC: Husky zenoh logged `Remote ROS 2 bridge left` (Go2 bridge gone — stopped by hand or Wi-Fi drop; to be checked).
- Whether the dog physically walked was not recorded; the dog has to be in balance stand for `Move()` to take effect.

### 4.11 2026-10-13 — `/go2/command` verified end to end
- Setup change: Huihang's Mac wired directly to the Go2 (`ssh unitree@192.168.123.18`); Go2 clock was correct today (14:13 EDT).
- On the Go2: `git pull` (already up to date, `grep -c onCommand` = 2), clean rebuild of `go2_bridge` (23.8 s), started with `ROS_DOMAIN_ID=1` → `go2_bridge up on interface 'eth0'`.
- **First attempt:** Husky `ros2 topic pub --once -w 1 /go2/command … stand_up` printed `Waiting for at least 1 matching subscription(s)...` forever. The Husky zenoh log showed the local `Route Publisher (ROS:/go2/command -> Zenoh:go2/command) created`, but **no `Remote bridge … announces` lines** → the Go2-side zenoh bridge (terminal B) had not been started.
- **Fix:** started terminal B on the Go2. Its log then shows `Remote bridge 869d6a7c… announces Publisher go2/command - Allowed` each time the Husky publishes.
- **Result:** `stand_up`, `stand_up`, `stand_down`, `stand_up` sent from the Husky (18:25 UTC) were each published immediately (`publishing #1`) and **the dog stood up / lay down accordingly**.

![2026-10-13 test](images/2026-10-13_go2_command_test.png)

---

## 5. Troubleshooting quick reference

| Symptom | Likely cause | Fix |
|---|---|---|
| `ros2: command not found` on Go2 | chose `2` (noetic) at login | log in again, choose `1` |
| `Temporary failure resolving …` on Go2 | default route via `eth0` | `sudo ip route del default via 192.168.123.1 dev eth0` |
| apt / git SSL errors on Go2 | clock reset to 1970 | `sudo date -s "…"` |
| SSH via Wi-Fi: `No route to host` | Go2 `wlan0` dropped | wired SSH to `192.168.123.18`; `nmcli dev status`; reboot Jetson if needed |
| `bad_alloc caught` | ROS on domain 0 on the Go2 | `export ROS_DOMAIN_ID=1` (check `echo $ROS_DOMAIN_ID`) |
| `ddsi_sertype` compile error | ROS CycloneDDS headers first | use the repo's CMakeLists (BEFORE include + RPATH) |
| `unrecognized arguments: --once` | Foxy CLI | drop `--once` on the Go2 |
| zenoh `No such file or directory` | wrong config path | check path, use Tab completion |
| `/go2/*` missing on Husky | a bridge or `go2_bridge` not running, or Go2 `ROS_DOMAIN_ID` ≠ 1 | check all 4 terminals |
| service call hangs, zenoh "less than 20 bytes" | services not bridged with Fast DDS | use `/go2/command` topic |
| `Waiting for at least 1 matching subscription(s)` on Husky | Go2 zenoh bridge (B) or `go2_bridge` (A) not running | start them; Husky zenoh log must show `Remote bridge … announces` |
| `cmd_vel timeout -> StopMove` while publishing | Wi-Fi gaps > `cmd_timeout` | measure `ros2 topic hz`; raise `cmd_timeout` |

---

## 6. Open issues / next steps

1. ~~Rebuild with the `/go2/command` version; test `stand_up` / `stand_down`~~ — done 2026-10-13. Next: walking via `/go2/cmd_vel` while standing.
2. Investigate `cmd_vel` delivery gaps (on the Go2: `ros2 topic hz /go2/cmd_vel` with `ROS_DOMAIN_ID=1`); consider `cmd_timeout:=1.0`.
3. Check why the Go2 zenoh bridge left at 21:16:41 UTC; monitor `wlan0` stability; make `power_save off` persistent.
4. Make the route fix permanent (`ipv4.never-default`) and set up time sync on the Go2.
5. Launch script for the Go2 (go2_bridge + zenoh) and the Husky (zenoh), possibly started on boot.
6. Verify odom twist sign/frame on the real robot; plan a shared frame between `go2/odom` and the Husky's TF for joint tasks.
7. Security: restrict who can connect to port 7447 on PSU-IOT.

---

## Appendix A — `go2_bridge/src/go2_bridge.cpp`

```cpp
// go2_bridge: ROS 2 node (runs on the Go2's Jetson, ROS 2 Foxy) that talks to the Go2
// over unitree_sdk2 (DDS domain 0 on the dog's internal 192.168.123.0/24 network).
//
//   Sub:  cmd_vel   (geometry_msgs/Twist)  -> SportClient::Move(vx, vy, wz)
//   Sub:  command   (std_msgs/String)      -> "stand_up" | "stand_down" | "recovery_stand" | "stop"
//   Pub:  odom      (nav_msgs/Odometry)    <- rt/sportmodestate
//   Srv:  stand_up / stand_down / stop     (std_srvs/Trigger)  -- local use only:
//         services do not cross zenoh-bridge-ros2dds when the RMW is Fast DDS,
//         so remote machines (the Husky) use the `command` topic instead.
//
// Run with ROS_DOMAIN_ID != 0 (domain 0 is the dog's own DDS traffic), e.g.
//   ROS_DOMAIN_ID=1 ros2 run go2_bridge go2_bridge --ros-args -r __ns:=/go2 -p network_interface:=eth0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace std::chrono_literals;
using SportState = unitree_go::msg::dds_::SportModeState_;

class Go2Bridge : public rclcpp::Node
{
public:
  Go2Bridge() : Node("go2_bridge")
  {
    iface_       = declare_parameter<std::string>("network_interface", "");
    max_vx_      = declare_parameter<double>("max_vx", 0.5);    // m/s
    max_vy_      = declare_parameter<double>("max_vy", 0.3);    // m/s
    max_wz_      = declare_parameter<double>("max_wz", 0.8);    // rad/s
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5); // s, stop if no cmd_vel
    odom_frame_  = declare_parameter<std::string>("odom_frame", "go2/odom");
    base_frame_  = declare_parameter<std::string>("base_frame", "go2/base_link");
    double odom_rate = declare_parameter<double>("odom_rate", 50.0); // Hz

    if (iface_.empty()) {
      throw std::runtime_error("parameter 'network_interface' is required (the NIC on 192.168.123.x)");
    }

    // 1) DDS for the Unitree side. Must be initialised before any client/subscriber.
    unitree::robot::ChannelFactory::Instance()->Init(0, iface_);

    // 2) High-level sport client (RPC). Short timeout so a dead link can't block the executor long.
    sport_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_->SetTimeout(1.0f);
    sport_->Init();

    // 3) State subscriber (runs on an SDK thread; we only copy the latest message here).
    state_sub_.reset(new unitree::robot::ChannelSubscriber<SportState>("rt/sportmodestate"));
    state_sub_->InitChannel(std::bind(&Go2Bridge::onState, this, std::placeholders::_1), 1);

    // ROS side
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Go2Bridge::onCmdVel, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, std::bind(&Go2Bridge::onCommand, this, std::placeholders::_1));

    // Explicit callback types: Foxy's rclcpp cannot deduce the signature of generic (auto) lambdas.
    using Trigger = std_srvs::srv::Trigger;
    srv_stand_up_ = create_service<Trigger>("stand_up",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = (sport_->StandUp() == 0); res->message = "StandUp"; });
    srv_stand_down_ = create_service<Trigger>("stand_down",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = (sport_->StandDown() == 0); res->message = "StandDown"; });
    srv_stop_ = create_service<Trigger>("stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        moving_ = false; res->success = (sport_->StopMove() == 0); res->message = "StopMove"; });

    odom_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / odom_rate), std::bind(&Go2Bridge::publishOdom, this));
    watchdog_timer_ = create_wall_timer(50ms, std::bind(&Go2Bridge::watchdog, this));

    RCLCPP_INFO(get_logger(), "go2_bridge up on interface '%s'", iface_.c_str());
  }

  ~Go2Bridge() override
  {
    if (sport_) sport_->StopMove();
  }

private:
  void onState(const void * msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    state_ = *static_cast<const SportState *>(msg);
    have_state_ = true;
  }

  void publishOdom()
  {
    SportState s;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_state_) return;
      s = state_;
    }

    const double roll  = s.imu_state().rpy()[0];
    const double pitch = s.imu_state().rpy()[1];
    const double yaw   = s.imu_state().rpy()[2];

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = s.position()[0];
    odom.pose.pose.position.y = s.position()[1];
    odom.pose.pose.position.z = s.position()[2];

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // sportmodestate velocity is treated as odom/world-frame; Odometry twist is in the
    // child (body) frame, so rotate by -yaw. Verify the sign on the real robot.
    const double vxw = s.velocity()[0], vyw = s.velocity()[1];
    odom.twist.twist.linear.x =  std::cos(yaw) * vxw + std::sin(yaw) * vyw;
    odom.twist.twist.linear.y = -std::sin(yaw) * vxw + std::cos(yaw) * vyw;
    odom.twist.twist.linear.z = s.velocity()[2];
    odom.twist.twist.angular.z = s.yaw_speed();

    odom_pub_->publish(odom);
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const float vx = static_cast<float>(std::clamp(msg->linear.x,  -max_vx_, max_vx_));
    const float vy = static_cast<float>(std::clamp(msg->linear.y,  -max_vy_, max_vy_));
    const float wz = static_cast<float>(std::clamp(msg->angular.z, -max_wz_, max_wz_));

    last_cmd_ = now();
    if (vx == 0.f && vy == 0.f && wz == 0.f) {
      if (moving_) { sport_->StopMove(); moving_ = false; }
      return;
    }
    int32_t ret = sport_->Move(vx, vy, wz);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Move() returned %d (is the dog standing?)", ret);
    }
    moving_ = true;
  }

  void onCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & c = msg->data;
    int32_t ret = -1;
    if (c == "stand_up") {
      // Locked stand, then unlock into balance stand so Move() is accepted.
      ret = sport_->StandUp();
      std::this_thread::sleep_for(1500ms);
      if (ret == 0) ret = sport_->BalanceStand();
    } else if (c == "recovery_stand") {
      ret = sport_->RecoveryStand();
    } else if (c == "stand_down") {
      moving_ = false;
      sport_->StopMove();
      ret = sport_->StandDown();
    } else if (c == "stop") {
      moving_ = false;
      ret = sport_->StopMove();
    } else {
      RCLCPP_WARN(get_logger(), "unknown command '%s' (use stand_up | stand_down | recovery_stand | stop)", c.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "command '%s' -> ret=%d", c.c_str(), ret);
  }

  void watchdog()
  {
    if (moving_ && (now() - last_cmd_).seconds() > cmd_timeout_) {
      sport_->StopMove();
      moving_ = false;
      RCLCPP_INFO(get_logger(), "cmd_vel timeout -> StopMove");
    }
  }

  // params
  std::string iface_, odom_frame_, base_frame_;
  double max_vx_, max_vy_, max_wz_, cmd_timeout_;

  // unitree
  std::unique_ptr<unitree::robot::go2::SportClient> sport_;
  unitree::robot::ChannelSubscriberPtr<SportState> state_sub_;
  std::mutex state_mtx_;
  SportState state_;
  bool have_state_ = false;

  // ros
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stand_up_, srv_stand_down_, srv_stop_;
  rclcpp::TimerBase::SharedPtr odom_timer_, watchdog_timer_;
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
  bool moving_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2Bridge>());
  rclcpp::shutdown();
  return 0;
}
```

## Appendix B — `go2_bridge/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(go2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# unitree_sdk2 installed with: cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && sudo make install
set(UNITREE_PREFIX "/opt/unitree_robotics")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_PREFIX}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_bridge src/go2_bridge.cpp)
ament_target_dependencies(go2_bridge rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2)
target_link_libraries(go2_bridge unitree_sdk2)

# unitree_sdk2 bundles its own (newer) CycloneDDS, while ROS (Foxy especially) ships an
# older CycloneDDS in /opt/ros/<distro>/include. Put the SDK's headers FIRST so that
# dds/dds.h and ddscxx resolve to the SDK's copy, not ROS's.
target_include_directories(go2_bridge BEFORE PRIVATE
  ${UNITREE_PREFIX}/include
  ${UNITREE_PREFIX}/include/ddscxx)

# Same at runtime: load the SDK's libddsc/libddscxx, not ROS's. DT_RPATH (old-style) is
# searched before LD_LIBRARY_PATH, which sourcing ROS fills with /opt/ros/<distro>/lib.
set_target_properties(go2_bridge PROPERTIES
  BUILD_RPATH "${UNITREE_PREFIX}/lib"
  INSTALL_RPATH "${UNITREE_PREFIX}/lib")
target_link_options(go2_bridge PRIVATE "-Wl,--disable-new-dtags")

install(TARGETS go2_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Appendix C — `go2_bridge/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>go2_bridge</name>
  <version>0.1.0</version>
  <description>ROS 2 bridge for a Unitree Go2 via unitree_sdk2 (runs on the Go2 Jetson)</description>
  <maintainer email="hql5777@psu.edu">Huihang Liu</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>tf2</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Appendix D — `go2_bridge/config/go2_zenoh.json5`

```json5
// zenoh-bridge-ros2dds config shared by the Husky and the Go2.
// Only /go2/* crosses the Wi-Fi; Husky's own topics and the Go2's internal rt/* stay local.
{
  plugins: {
    ros2dds: {
      allow: {
        publishers: ["/go2/.*"],
        subscribers: ["/go2/.*"],
        service_servers: ["/go2/.*"],
        service_clients: ["/go2/.*"],
      },
    },
  },
}
```

## Appendix E — `go2_tools/go2_state_test/main.cpp` (SDK-only test)

```cpp
// Minimal read-only test: unitree_sdk2 only, no ROS.
// Subscribes to rt/sportmodestate for 10 s and prints the position.
//   ./go2_state_test eth0
#include <atomic>
#include <iostream>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using SportState = unitree_go::msg::dds_::SportModeState_;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " <network_interface, e.g. eth0>" << std::endl;
    return 1;
  }
  std::cout << "init DDS on " << argv[1] << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  std::atomic<int> n{0};
  unitree::robot::ChannelSubscriberPtr<SportState> sub;
  sub.reset(new unitree::robot::ChannelSubscriber<SportState>("rt/sportmodestate"));
  sub->InitChannel([&n](const void * msg) {
    const auto & s = *static_cast<const SportState *>(msg);
    if (n++ % 50 == 0) {
      std::cout << "pos x=" << s.position()[0] << " y=" << s.position()[1]
                << " yaw=" << s.imu_state().rpy()[2] << std::endl;
    }
  }, 1);

  std::cout << "listening 10 s..." << std::endl;
  sleep(10);
  std::cout << "received " << n.load() << " messages" << std::endl;
  return 0;
}
```

Build and run on the Go2 (no ROS needed):

```bash
cd ~/go2_ws/src/HuskyA200/go2_tools/go2_state_test && mkdir -p build && cd build
cmake .. && make
env -u CYCLONEDDS_URI ./go2_state_test eth0
```

## Appendix F — `go2_tools/go2_state_test/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.16)
project(go2_state_test LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(UNITREE_PREFIX "/opt/unitree_robotics")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_PREFIX}")
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_state_test main.cpp)
target_link_libraries(go2_state_test unitree_sdk2)
target_include_directories(go2_state_test BEFORE PRIVATE
  ${UNITREE_PREFIX}/include ${UNITREE_PREFIX}/include/ddscxx)
# Always load the SDK's libddsc, even if ROS is sourced
set_target_properties(go2_state_test PROPERTIES BUILD_RPATH "${UNITREE_PREFIX}/lib")
target_link_options(go2_state_test PRIVATE "-Wl,--disable-new-dtags")
```
