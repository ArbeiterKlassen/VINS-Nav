# Go1 bring-up — 实机验证版操作说明

**状态（2026-10-17）**：相机 H.264 端到端已验证；低层 500Hz 状态解码已验证；
`.161:8082` 高层服务的判读已纠正。证据、事件时间线与全部纠错见
[`FINDINGS-2026-10-11.md`](FINDINGS-2026-10-11.md)；机器人自带 ROS 图的只读流程见
[`ROS_BRINGUP.md`](ROS_BRINGUP.md)；时间同步见 [`TIMESYNC.md`](TIMESYNC.md)。

> 旧版本文档中 **"推流端口 9201–9205"** 与 **"8082 服务没跑"** 两处结论已被实机
> 证伪，以本文件与 FINDINGS 为准。

---

## 0. 安全 SOP（每次动手前先读）

1. **默认不带任何客户端运行**；SDK/控制类客户端**用完即断**（不要留后台 500Hz 流）。
2. 启动任何**向运动系统发包**的客户端（ros_udp、8007 轮询、控制测试）前**先打招呼**；
   狗在地面、有人看护、有人手放电源开关。
3. 单客户端：`.161:8082` 同一时刻只服务一个客户端（connect() 语义）。其他源端口
   探测会收到 **ICMP port-unreachable —— 不代表服务没跑**。
4. `.13` 磁盘 81%：一切产物放 `/dev/shm`（RAM），不落盘。
5. 相机自启节点只 **kill 不删**；恢复脚本随时可用（`camera_nodes.sh`）。

## 1. 网络（已验证）

| 接口 | 地址 | 用途 |
|---|---|---|
| `ens33` | 192.168.38.134/24 | VMnet NAT — 外网 |
| `ens37` | **192.168.123.101/24** | 机器人网段直连（相机接收地址） |

| 端点 | 地址 | 说明 |
|---|---|---|
| 运动控制板 | `192.168.123.10` | UDP **8007** 低层状态/控制 |
| sport 高层服务 | `192.168.123.161` | UDP **8082**（单客户端）；ROS master **11311** |
| Jetson | `.13 / .14 / .15` | `ssh unitree@<ip>` 免密；相机分配见 §2 |

## 2. 相机 H.264 链路（已验证，端口 = 9000 + 位置号）

| 板 | 相机 | 设备节点 | 备注 |
|---|---|---|---|
| `.13` (unitree-desktop) | **camera1 = face（前视）** | `/dev/video1` | 本次演示使用 |
| `.13` | camera2 = chin（下巴） | `/dev/video0` | |
| `.14` (unitree-desktop) | camera3 = left / camera4 = right | video1 / video0 | 侧向双目对 |
| `.15` (nx) | camera5 = belly（腹部） | `/dev/video0` | |

- **端口纠正（实测）**：本世代板载 SDK 的 `startCapture(true,false)` 推流到
  **9000 + 相机位置号**（依据 `StereoCameraCommon.hpp:567` 文档）。face → **9001**。
  示例注释里的 9201–9205 属另一套机制，实测**无包**，不要混用。
- 推流原理：Jetson 本机 V4L2 采集 → `omxh264enc` 硬编码 → RTP/UDP 推给接收端。
  构建/运行配方见 [`stage/README.md`](stage/README.md)（在 `.13` 上全 `/dev/shm`，
  零磁盘写入；需两处适配：3 参 `getRectStereoFrame(l,r,feim)`、`waitKey→usleep`）。
- 发送端配置 `IpLastSegment: 101` → 推给 VM `192.168.123.101`。
- **流程**：
  ```bash
  # 1) 释放相机（只 kill 不删；record 输出到 stdout，板子不落盘）
  ssh unitree@192.168.123.13 'bash -s record -' < camera_nodes.sh > cam_13.json
  scp cam_13.json unitree@192.168.123.13:/dev/shm/cam_nodes_record.json
  ssh unitree@192.168.123.13 'bash -s kill /dev/shm/cam_nodes_record.json' < camera_nodes.sh
  # 2) 标定读取（板载二进制，cwd 中含 stereo_camera_config.yaml）
  ssh unitree@192.168.123.13 \
    'cd /dev/shm/ucsdk_run && /home/unitree/Unitree/sdk/UnitreeCameraSdk/bins/example_getCalibParamsFile'
  # 3) 编译并运行推流器（见 stage/README.md）
  # 4) VM 端验包 + 解码
  python3 udp_probe.py --ports 9001 --seconds 10
  gst-launch-1.0 -e udpsrc address=192.168.123.101 port=9001 \
    caps="application/x-rtp,media=video,encoding-name=H264" \
    ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert \
    ! videorate ! video/x-raw,framerate=2/1 ! pngenc \
    ! multifilesink location=front_%05d.png
  ```
- **停止推流（SOP：用完必停）**：`kill $(cat /dev/shm/ucsdk_run/put.pid)`；
  核实进程用 `ps -eo pid,args | grep -F ./example_putImagetrans`
  （**`pgrep` 因 comm 15 字符截断会误报"进程不存在"**）。
- **恢复相机节点**：`camera_nodes.sh restore`（autostart 入口只含 camerarosnode，
  **不含 imageai**）；全量恢复用 record 回放（`GO1_CAM_ENTRY=/nonexistent`）或重启板子。
  断电重启会自动复原自启节点。
- 证据帧：`cam_evidence/front_*.png`（44 帧 928×800，**不入库**；主机侧亦已取回副本）。

## 3. 低层状态（`.10:8007`，500Hz，已验证）

- 请求-应答：发 64B `'x'` 填充包 → 回 820B 压缩 LowState（帧头 `FE EF FF`）。
- 用法（**SOP：属发包，用前打招呼**）：
  ```bash
  python3 lowstate_rx.py --seconds 4 --hz 500 --csv /tmp/imu.csv
  ```
  实测 PASS：500.6 帧/s、820B 恒定、tick 1002 ms/s、四元数范数 1.000000。
- 关键偏移：IMU @22（quat/gyro/accel/rpy，f32；temp@74）；12 电机槽 @75+32k
  （mode@+0、q f32@+1、dq f32@+5、temp@+23）；**tick u32 @755（单位 ms）**。
  ddq/tauEst 为量化字段（整数 / k·2⁻⁸），工具暂按原始字节读取。
- 用途：给 VIO 的 500Hz IMU（`--csv` 直接落表）。

## 4. 高层状态（`.161:8082`，单客户端）

- `/high_state` 平时无数据的原因：`ros_udp` 只在 `/high_cmd` 回调里发布状态。
- 采样（**属发包，用前打招呼**）：
  ```bash
  rostopic echo -n1 /high_state        # 终端1（先订阅）
  python3 pub_high_cmd_mirror_once.py  # 终端2（逐位镜像当前流，无行为变化）
  ```
- 2026-10-11 实测：机器人 mode=7（damping）、bodyHeight≈0.007 m、平躺；IMU 水平。
  **"站起来"未达成**——mode 0 对 damping 无效；下一步候选 mode 6/8（需打招呼+看护）。
  另注意遥控器锁定态（L2+R2）。
- 服务端自清：客户端断开后新探测立即回 **1087B**（= sizeof(HighState)，**未压缩**）。

## 5. 机器人自带 ROS 图（只读）

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=192.168.123.101
rosnode list; rostopic list
```
- 常驻节点：`/camera1` `/camera3` `/camera4` `/camera5` `/node_lcm`
  `/node_obstacle_avoidance` `/node_obstacle_traverse` `/ros2udp_motion_mode_adv`
  `/ukd_triple_2_goal` `/ukd_triple_udp_node` `/rosout`。
- `/ros2udp/odom` ≈200 Hz（ukd 桥；空闲时全零）；`/joint_states`、`/lcm_node/*`
  可能静默；`/ukd_triple/state` 为 `a2_msgs/UKDState`（本机未装，无法解码）。

## 6. 相机节点管理（`camera_nodes.sh`）

```bash
./camera_nodes.sh status
./camera_nodes.sh record  ~/go1_cam_backup.json   # 或 record - 输出到 stdout（推荐，板子不落盘）
./camera_nodes.sh kill    ~/go1_cam_backup.json   # 先校验 argv，再 SIGTERM→SIGKILL
./camera_nodes.sh restore ~/go1_cam_backup.json   # 优先板载 autostart 入口
```
- matcher：`point_cloud_node example_point live_human_pose mqttControlNode`。
- 本仓库留有三台板的 record：`cam_13.json` / `cam_14.json` / `cam_15.json`。

## 7. 文件索引

| 文件 | 说明 |
|---|---|
| `FINDINGS-2026-10-11.md` | 现场发现 / 纠错 / 事件闭环 / SOP（**先读**） |
| `stage/` | 推流配置 + 复制即用的构建说明（第三方示例源码不入库） |
| `lowstate_rx.py` | 8007 低层状态解码器（IMU CSV） |
| `pub_high_cmd_mirror_once.py` | 触发 `/high_state` 采样 |
| `camera_nodes.sh` | 相机节点 kill / restore 框架 |
| `udp_probe.py` | 相机流验包探针 |
| `setup_go1_env.sh` | 本 VM 的 ROS / catkin 环境 |
| `cam1_face_calib_2026-10-11.yaml` | face 相机出厂标定（PosNumber=1，基线≈25.09 mm） |
| `TASKS.md` | 任务清单（已完成/待办/停机原因） |
| `stage/stereo_tcp_sender.cc` | 双目 TCP+JPEG 发送端（`.13` 编译） |
| `go1_stereo_receiver.py` | 双目接收端（ROS 图像 + camera_info / 存 PNG） |
| `go1_imu_publisher.py` | 8007 IMU → `/go1/imu`（500 Hz，已获准启用） |
| `record_stereo_imu.sh` | 一键录制「双目 + IMU」rosbag（手柄操控时使用） |
| `make_stereo_bag.py` | PNG 对 [+IMU CSV] → rosbag（离线 SLAM 输入） |
| `run_slam_offline.sh` | 离线 SLAM（RTAB-Map stereo / `--ov-config` OpenVINS） |
| `stereo_depth_demo.py` | 双目测距演示（SGBM 视差 → 深度图） |
| `go1_stand_up.py` | 站立脚本（HighCmd mode 6/8；`--confirm` 才发送） |

## 8. 双目 / IMU / 离线 SLAM / 站立（2026-10-17 新增）

- **双目**：SDK 的 UDP H.264 推流实测**只发单目**（Transmode 1/3 无效）；双目走本地
  API（`getStereoFrame`/`getRectStereoFrame`，左右目各 928×800、带 µs 时间戳；
  **校正输出 180° 倒置**，VIO/建图不受方向影响），经自建 TCP+JPEG 桥回传：
  `.13` 上 `stage/stereo_tcp_sender.cc` → VM `go1_stereo_receiver.py`。
- **IMU**（已获同意启用）：`go1_imu_publisher.py` 以 500 Hz 轮询 `.10:8007`
  （只发 64B 非法填充触发包，无控制副作用），发布 `/go1/imu`，可同时写 CSV。
- **录制**（运动来源 = 遥控手柄；**录制期间不得启动任何 8082 客户端**，sport
  单客户端语义留给手柄）：
  ```bash
  source /opt/ros/noetic/setup.bash
  ./record_stereo_imu.sh --out /tmp/go1_rec.bag --seconds 90
  ```
- **离线 SLAM**：`make_stereo_bag.py` → `run_slam_offline.sh --bag B --out D`
  （无 IMU 走 RTAB-Map stereo；`--ov-config` 走 OpenVINS）。建图核心已用
  `house_full.bag` 的**真实运动数据**复跑验证（VIO TF + depth → 占用栅格图）。
- **站立脚本**（运动指令）：`go1_stand_up.py`（mode 6/8，auto 6→8；CRC 已用官方
  SDK 静态库逐字节校验）——必须 `--confirm`，按 SOP 有人看护时使用。
- **标定**：暂用估计内参（hFov≈90° ⇒ fx=width/2，基线≈25.09 mm）；
  `stereo_depth_demo.py` 可出视差/深度图与距离估计。后续用 OpenCV fisheye
  棋盘格（或 Kalibr）实测替换，见 `TASKS.md`。
