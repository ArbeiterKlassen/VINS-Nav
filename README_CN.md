# VINS-Nav: 伪双目视觉惯性 SLAM + 导航

> [English](README.md)

基于 Gazebo TurtleBot3 house 环境的端到端视觉惯性 SLAM 与导航系统。
核心创新是**伪双目技术**：通过单目 RGB-D 相机深度图生成虚拟右目图像，
使 OpenVINS 能够以双目模式运行，获得尺度正确的里程计输出。

**尺度精度：0.991x**（单目模式发散 200×）。  
**3.1m 轨迹漂移：2.8cm（0.9%）。**  
**方位误差：0.0°。**

---

## 技术博客

[伪双目 VIO + RTAB-Map 全链路 SLAM 系统搭建](https://soyorin.work/articles/000031.html)

---

## 目录

1. [系统架构](#系统架构)
2. [ROS 话题](#ros-话题)
3. [TF 树](#tf-树)
4. [环境与安装](#环境与安装)
5. [完整复现教程](#完整复现教程)
6. [脚本说明](#脚本说明)
7. [配置参数](#配置参数)
8. [文件清单](#文件清单)
9. [性能指标](#性能指标)
10. [故障排除](#故障排除)

---

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                    Gazebo TurtleBot3 House (12m×12m)               │
│                                                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ RGB 相机 │  │ 深度相机 │  │   IMU    │  │  激光扫描仪      │  │
│  │ 640×480  │  │ 640×480  │  │  333Hz   │  │  360°, 5Hz       │  │
│  │  28Hz    │  │  28Hz    │  │          │  │                  │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
└───────┼──────────────┼────────────┼──────────────────┼───────────┘
        │              │            │                  │
        ▼              ▼            ▼                  │
 ┌──────────────┐      │    ┌────────────────┐         │
 │ Pseudo-Stereo│◄─────┘    │   OpenVINS     │         │
 │ 视差投影     │──右目──►│   双目 VIO     │         │
 │ d = fx*B/Z   │  图像    │  max_cameras=2 │         │
 └──────────────┘          └───────┬────────┘         │
                                   │ /ov_msckf/       │
                                   │   odomimu        │
                                   ▼                  │
                          ┌────────────────┐          │
                          │   RTAB-Map     │◄─────────┘
                          │   RGBD SLAM    │  /scan
                          └───────┬────────┘
                                  │ /rtabmap/grid_map
                                  ▼
                          ┌────────────────┐
                          │  导航栈        │
                          │  map_server    │
                          │  AMCL          │
                          │  move_base     │
                          └────────────────┘
```

### 数据流

```
传感器层 (Gazebo):
  /camera/rgb/image_raw       [sensor_msgs/Image]      640×480, ~28Hz
  /camera/depth/image_raw     [sensor_msgs/Image]      640×480, ~28Hz
  /camera/rgb/camera_info     [sensor_msgs/CameraInfo]  fx=402.3, cx=320.5
  /imu                        [sensor_msgs/Imu]         ~333Hz
  /scan                       [sensor_msgs/LaserScan]  360°, 5Hz, 0.12-3.5m
  /clock                      [rosgraph_msgs/Clock]    仿真时间

处理层:
  pseudo_stereo.py:
    输入:  /camera/rgb/image_raw, /camera/depth/image_raw, /camera/rgb/camera_info
    输出:  /camera/right/image_raw, /camera/right/camera_info

  OpenVINS (run_subscribe_msckf):
    输入:  /camera/rgb/image_raw, /camera/right/image_raw, /imu
    输出:  /ov_msckf/odomimu, /ov_msckf/poseimu, /ov_msckf/pathimu

  dyn_odom_tf.py:
    输入:  /ov_msckf/odomimu
    输出:  /tf (global→odom)

  RTAB-Map:
    输入:  /camera/rgb/image_raw, /camera/depth/image_raw,
           /camera/rgb/camera_info, /ov_msckf/odomimu
    输出:  /rtabmap/grid_map, /rtabmap/cloud_map, /rtabmap/info,
           /rtabmap/mapData, /rtabmap/mapGraph

导航层:
  map_server:  输入: (文件)  输出: /map (静态, 锁存)
  AMCL:        输入: /map, /scan  输出: /amcl_pose, /tf (map→odom)
  move_base:   输入: /map, /scan, /tf  输出: /cmd_vel
```

---

## ROS 话题

### 本系统发布的话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/camera/right/image_raw` | sensor_msgs/Image | ~28Hz | 深度图投影生成的虚拟右目 |
| `/camera/right/camera_info` | sensor_msgs/CameraInfo | ~28Hz | 右目相机内参（含基线修正的 P 矩阵） |
| `/ov_msckf/odomimu` | nav_msgs/Odometry | ~300Hz | VIO 里程计 (frame: global, child: imu) |
| `/ov_msckf/poseimu` | geometry_msgs/PoseWithCovarianceStamped | ~28Hz | 带协方差的位姿估计 |
| `/ov_msckf/pathimu` | nav_msgs/Path | ~28Hz | 历史轨迹 |
| `/rtabmap/grid_map` | nav_msgs/OccupancyGrid | 变化时 | 2D 全局占据栅格 |
| `/rtabmap/cloud_map` | sensor_msgs/PointCloud2 | 变化时 | 3D 点云地图 |
| `/rtabmap/info` | rtabmap_msgs/Info | ~1Hz | 地图统计 (refId, loopClosureId) |
| `/rtabmap/mapData` | rtabmap_msgs/MapData | 变化时 | 完整地图图谱（含 mapToOdom） |
| `/tf` (global→odom) | tf2_msgs/TFMessage | 30Hz | dyn_odom_tf.py 发布的动态 VIO 位姿 |

### 来自 Gazebo 的订阅话题

| 话题 | 类型 | 频率 | 消费者 |
|------|------|------|--------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | ~28Hz | pseudo_stereo, OpenVINS, RTAB-Map |
| `/camera/depth/image_raw` | sensor_msgs/Image | ~28Hz | pseudo_stereo, RTAB-Map |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | ~28Hz | pseudo_stereo, RTAB-Map |
| `/imu` | sensor_msgs/Imu | ~333Hz | OpenVINS |
| `/scan` | sensor_msgs/LaserScan | ~5Hz | AMCL, move_base |
| `/clock` | rosgraph_msgs/Clock | ~100Hz | 全部节点 (use_sim_time=true) |

---

## TF 树

```
                           ┌─────────────────┐
                           │       map       │  (RTAB-Map SLAM 结果)
                           └────────┬────────┘
                                    │
                           ┌────────▼────────┐
                           │     global      │  (OpenVINS 世界坐标系)
                           └────────┬────────┘
                                    │ dyn_odom_tf.py (30Hz, 从 /ov_msckf/odomimu)
                           ┌────────▼────────┐
                           │      odom       │
                           └────────┬────────┘
                                    │ static_tf_publisher (0, 0, -0.078)
                           ┌────────▼────────┐
                           │  base_footprint │
                           └────────┬────────┘
                                    │ robot_state_publisher (URDF)
                           ┌────────▼────────┐
                           │    base_link    │
                           └──┬──────┬──────┘
                              │      │
              ┌───────────────┘      └───────────────┐
              ▼                                      ▼
   ┌──────────────────┐                    ┌──────────────────┐
   │    imu_link      │                    │   camera_link    │
   │  (0, 0, 0.068)   │                    │ (0.064,-0.065,   │
   └──────────────────┘                    │   0.094)          │
                                           └────────┬─────────┘
                                                    │
                                           ┌────────▼─────────┐
                                           │ camera_rgb_frame  │
                                           │ (0.005, 0.018,    │
                                           │   0.013)          │
                                           └────────┬─────────┘
                                                    │ rpy(-1.57,0,-1.57)
                                           ┌────────▼─────────┐
                                           │camera_rgb_optical │
                                           │     _frame        │
                                           └──────────────────┘
```

**关键设计决策：**
- `global→odom` 是**动态**的（来自 VIO 里程计）。此处不能用静态 TF，否则 RTAB-Map 无法跟踪机器人运动。
- `odom→base_footprint` 是静态的 (0,0,-0.078)。IMU 在 base_footprint 上方 0.078m。
- `base_footprint` **只能有一个父帧**。绝对不能同时发布 `imu→base_footprint` 和 `odom→base_footprint`。
- OpenVINS 内置的 TF 发布器（`publish_global_to_imu_tf`）已禁用，避免 333Hz 的 TF 缓冲区洪水。

---

## 环境与安装

### 系统要求
- Ubuntu 20.04
- ROS Noetic (完整桌面版)
- Python 3.8+

### ROS 包
```bash
sudo apt install -y \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-teleop \
  ros-noetic-rtabmap-ros \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-tf2-ros
```

### Python 依赖
```bash
pip3 install numpy opencv-python pyyaml
```

### OpenVINS 编译
```bash
mkdir -p ~/catkin_ws_ov/src
cd ~/catkin_ws_ov/src
git clone https://github.com/rpng/open_vins.git
cd ~/catkin_ws_ov
catkin_make -j4
source devel/setup.bash
```

### VINS-Nav 文件部署
将本仓库 `openvins/` 目录下的内容复制到 `~/catkin_ws_ov/src/open_vins/`，覆盖同名文件。
`scripts/` 目录下的独立脚本可放在任意位置运行。

```bash
cd ~/catkin_ws_ov
catkin_make  # 如有新 launch 文件需重新编译
```

---

## 完整复现教程

### 第一步：在线实时建图

```bash
# 终端 1：启动全系统
source ~/catkin_ws_ov/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch ov_msckf turtlebot3_house_stereo.launch

# 等待 OpenVINS 输出 "successful initialization"（约 45 秒）

# 终端 2：手动驾驶机器人
export TURTLEBOT3_MODEL=waffle
source ~/catkin_ws_ov/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# 终端 3：完成后导出地图
rosrun map_server map_saver map:=/rtabmap/grid_map -f ~/maps/house_live
```

### 第二步：录制 bag（离线处理用）

```bash
# 终端 1：启动系统
roslaunch ov_msckf turtlebot3_house_stereo.launch gui:=true

# 终端 2：录制 bag
rosbag record -O house.bak \
  /camera/rgb/image_raw \
  /camera/depth/image_raw \
  /camera/rgb/camera_info \
  /imu \
  /clock \
  /tf \
  /tf_static

# 终端 3：驾驶机器人
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# 在全屋行驶完成后 Ctrl-C 停止 bag 录制
```

### 第三步：离线处理 bag 建图

```bash
# 直接从 bag 建图（无需 RTAB-Map）
python3 scripts/build_map.py house.bak output_map

# 后处理
python3 scripts/postprocess_v5.py output_map.yaml output_map_final.pgm
```

### 第四步：导航

```bash
# 使用预建地图启动导航栈
roslaunch ov_msckf nav_clean.launch

# 发送导航目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}" \
  "pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}"
```

---

## 脚本说明

### `pseudo_stereo.py` — 虚拟右目生成器

通过深度图和视差公式生成虚拟右目相机图像。

**输入话题:** `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`
**输出话题:** `/camera/right/image_raw`, `/camera/right/camera_info`
**算法:** 对每个有效深度像素 Z，计算视差 d = fx * 基线 / Z。右图像素列 (u - d) 的像素值取自左图像素列 u。Z-buffer 通过 numpy 的 `unique` 实现遮挡剔除。Navier-Stokes inpainting 填补去遮挡空洞。
**基线:** 0.08m（对标 Intel RealSense D435）。
**性能:** 640×480 每帧 <10ms。

### `dyn_odom_tf.py` — 动态 TF 桥接

以 30Hz 频率从 VIO 里程计发布动态 TF 变换 `global→odom`。
RTAB-Map 依赖此变换跟踪机器人运动——静态 TF 会锁定机器人位置。

**输入:** `/ov_msckf/odomimu` (nav_msgs/Odometry)
**输出:** `/tf` (global→odom, 30Hz)

### `odom_tf_pub.py` — 替代 TF 桥接

功能同 dyn_odom_tf.py，但从里程计消息中直接读取 frame_id 和 child_frame_id。
用于在线 pipeline。

### `build_map.py` — 离线建图工具

直接从 ROS bag 构建 2D 占据栅格，完全绕过 RTAB-Map。

**输入:** 包含 `/tf`, `/camera/depth/image_raw`, `/camera/rgb/camera_info` 的 ROS bag 文件
**输出:** PGM + YAML 占据栅格地图
**算法:**
1. 从 bag 的 `/tf` (global→imu) 提取机器人位姿
2. 对每帧深度图，查找最近（100ms 容差内）的 TF 位姿
3. 相机坐标系投影：Xc = (u-cx)*Z/fx, Yc = (v-cy)*Z/fy
4. 旋转到 IMU 坐标系：Xi=Zc, Yi=-Xc, Zi=-Yc
5. 通过 IMU→global 四元数变换到世界坐标系
6. 累积点云，分类为墙面 (0.15m < Z < 3.0m) 或地面 (-0.5m < Z < 0.15m)
7. 栅格化：occ = 墙面主导单元 (gw*3 > gf), free = 清洁地面 (gf>20, gw==0)
8. VIO 漂移修正：对左半部分位姿（ox < -1.0）施加空间偏移

**参数:** `build_map.py <bag路径> [输出前缀] [max_time] [min_time] [corr_dx] [corr_dy]`
**示例:** `build_map.py house.bak map -1 -1 -3.2 -1.25`（全量 bag，带漂移修正）

### `postprocess_v5.py` — 地图后处理

清理和完善占据栅格地图。

**处理步骤：**
1. 去噪：移除邻域 <3 的被占格（深度传感器噪声）
2. 漂移过滤：移除水平拉伸（宽高比 >3:1）或小集群（<100px）的虚假墙
3. 边界填充：将邻近自由空间的未知格填充为被占（墙面补全）
4. 窄隙保护：跳过两侧 1-4px 范围内均有自由空间的格（门洞保留）
5. 手动漂移遮罩：用户标定的区域精确去噪

### `analyze_imu_noise.py` — IMU 噪声标定

对录制的 IMU bag 进行 Allan 方差分析，提取噪声密度和随机游走参数。

**用法:** `analyze_imu_noise.py <imu_bag>`

### `frontier_explore.py` — 自主探索器

检测占据栅格中的前沿单元（自由空间邻近未知的边界），使用 cv2.connectedComponents 聚类，向 move_base 发送探索目标。

**无 sklearn 依赖**——仅使用 OpenCV。
**输入:** `/rtabmap/grid_map`, `/move_base/status`
**输出:** `/move_base_simple/goal`

### `bounce_explore.py` — 反应式探索器

基于激光的反应式探索：直行直至受阻，找到扫描中最大角度空隙，朝该方向转动。窄空间检测防止振荡。无需地图或规划。

### `laser_circle.py` — 墙跟随探索器

基于激光扫描的右侧墙跟随行为。维持距右侧墙面约 0.6m 的距离。适用于房间系统覆盖。

---

## 配置参数

### `estimator_config.yaml`

| 参数 | 值 | 原因 |
|------|-----|------|
| `try_zupt` | true | 零速更新启用静态初始化 |
| `init_imu_thresh` | 0.05 | 高于实测 IMU 激励方差 (~0.026) |
| `init_max_features` | 200 | 更多特征提升初始化质量 |
| `num_pts` | 600 | 高特征数确保跟踪稳定 |
| `use_stereo` | true | 启用双目特征约束 |
| `max_cameras` | 2 | 左目（真实）+ 右目（虚拟） |
| `init_dyn_mle_max_iter` | 0 | MLE 已禁用——仿真环境特征不足 |
| `calib_cam_intrinsics` | false | 固定内参（Gazebo 已知值） |
| `calib_cam_timeoffset` | false | 仿真中无时间偏移 |
| `calib_cam_extrinsics` | false | 固定外参（已从 URDF 验证） |

### `kalibr_imu_chain.yaml`

| 参数 | 值 | 来源 |
|------|-----|------|
| `accelerometer_noise_density` | 0.00083 | 54s 静止 IMU 的 Allan 方差分析 |
| `gyroscope_noise_density` | 0.00083 | 同上 |
| `accelerometer_random_walk` | 1e-6 | 仿真无真实 bias 漂移 |
| `gyroscope_random_walk` | 1e-7 | 同上 |
| `update_rate` | 400 | 标称值（实际 ~333Hz） |

### `kalibr_imucam_chain.yaml`

| 参数 | cam0（左目） | cam1（右目） |
|------|-------------|-------------|
| `rostopic` | `/camera/rgb/image_raw` | `/camera/right/image_raw` |
| `resolution` | [640, 480] | [640, 480] |
| `intrinsics` | [403, 403, 320, 240] | [403, 403, 320, 240] |
| `distortion` | radtan, 全零 | radtan, 全零 |
| `T_cam_imu` 平移 | [-0.047, 0.039, -0.069] | [-0.127, 0.039, -0.069] |

cam0 与 cam1 之间的 0.08m 基线编码在平移差中：-0.047 - (-0.127) = 0.08。

### Gazebo 模型参数

| 参数 | 值 | 原因 |
|------|-----|------|
| `gaussianNoise` (IMU) | 0.015 | 匹配 noise_density=0.00083 @ 400Hz |
| `publishOdomTF` | false | 避免 base_footprint 双父帧冲突 |
| `updateRateHZ` (IMU) | 400 | 最大速率以获得良好 IMU 预积分 |
| `horizontal_fov` (相机) | 1.3439 rad | 标准 640×480 视场角 |

---

## 文件清单

### 对上游 OpenVINS 的修改

```
openvins/
├── config/
│   ├── nav/                          # 导航 costmap 和规划器参数（新增）
│   └── rgbd_stereo/                  # 双目 VIO 配置（新增）
│       ├── estimator_config.yaml     # ZUPT 初始化, 600 特征, MLE 禁用
│       ├── kalibr_imu_chain.yaml     # Allan 方差标定的噪声参数
│       └── kalibr_imucam_chain.yaml  # 2 相机双目配置, B=0.08m
└── ov_msckf/
    ├── gazebo_models/urdf/           # 自定义 TurtleBot3 模型（新增）
    │   ├── turtlebot3_waffle_openvins.urdf.xacro
    │   └── turtlebot3_waffle_openvins.gazebo.xacro
    ├── launch/                       # Launch 文件（新增）
    │   ├── turtlebot3_house_stereo.launch    # 主启动: Gazebo + 双目 OV + RTAB-Map
    │   ├── turtlebot3_house_openvins.launch  # 原始单目配置（保留）
    │   ├── rtabmap_db.launch                 # RTAB-Map 单独启动（含 odom remap）
    │   ├── offline_bag.launch                # 离线处理 pipeline
    │   ├── nav_clean.launch                  # AMCL + move_base
    │   └── explore.launch                    # 前沿探索
    └── scripts/                      # 新增脚本
        ├── pseudo_stereo.py          # 深度→右目相机投影
        ├── dyn_odom_tf.py            # 动态 global→odom TF
        ├── odom_tf_pub.py            # 里程计→TF 桥接
        ├── frontier_explore.py       # cv2 前沿检测
        ├── amcl_tf_pub.py            # AMCL→TF 桥接
        └── map_tf_broadcaster.py     # RTAB-Map→TF 桥接
```

### 独立工具

```
scripts/
├── build_map.py          # 离线 bag→地图 pipeline
├── postprocess_v5.py     # 地图后处理（去噪、填充、门洞保护）
├── postprocess_map.py    # 轻量地图后处理
├── analyze_imu_noise.py  # Allan 方差 IMU 噪声分析
├── offline_final.sh      # 一键离线处理脚本
├── bounce_explore.py     # 反应式激光探索
├── laser_circle.py       # 墙跟随探索
└── tf_debug.py           # TF2 缓存诊断工具
```

---

## 性能指标

| 指标 | 数值 | 备注 |
|------|------|------|
| 尺度比例 | 0.991x | 3.1m 轨迹 |
| 漂移 | 2.8cm (0.9%) | 3.1m 行程 |
| 方位误差 | 0.0° | 62° 指令转弯 |
| 初始化时间 | <1ms | ZUPT 静态初始化 |
| 跟踪特征数 | 190–276 | 优化后 |
| 地图覆盖 | 398×317 (19.9m×15.9m) | 后处理终版 |
| 障碍物格数 | 14,512 | 墙面 + 家具 |
| 伪双目延迟 | <10ms | 每帧 640×480 |
| OV 跟踪频率 | 100–300Hz | 随相机/IMU 比例变化 |
| Bag→地图处理 | ~5 分钟 | 35GB bag, 27M 点云 |

---

## 故障排除

### "failed static init: no accel jerk detected"
- 原因：IMU 噪声太低，静态初始化检测不到抖动。
- 解决：Gazebo IMU 插件中设置 `gaussianNoise` >= 0.01，同时 `init_imu_thresh` >= 实测 IMU 方差。

### "failed initialization in 0.0000 seconds"
- 原因：OpenVINS 特征数据库为空，或 `use_sim_time` 未设置。
- 解决：确认 `/clock` 正在发布，且 `rosparam get /use_sim_time` 返回 true。

### RTAB-Map 提示 "Did not receive data since 5 seconds"
- 原因：近似时间同步无法匹配相机话题。
- 解决：检查 `/camera/rgb/image_raw`、`/camera/depth/image_raw` 和 `/camera/rgb/camera_info` 的时间戳是否接近（0.1s 以内）。

### 地图始终 118×88，不随探索扩展
- 原因：静态 TF `global→base_footprint` 覆盖了动态里程计。
- 解决：用 `dyn_odom_tf.py`（动态 `global→odom`）+ 静态 `odom→base_footprint` 替代。

### 离线模式下 grid_map 不发布
- 原因：RTAB-Map 的 `odom` 话题 remap 不会创建订阅。
- 解决：离线建图请使用 `build_map.py` 替代 RTAB-Map。

### 运行约 3 分钟后机器人不响应 cmd_vel
- 原因：Gazebo 传感器在长 session 中退化。
- 解决：重启 Gazebo，使用 headless 模式 (`gui:=false`)，或录制 bag 离线处理。

### 地图右半部分错位
- 原因：VIO 在 10+ 分钟轨迹中累积漂移。
- 解决：使用 `build_map.py` 的空间修正参数：
  `python3 build_map.py house.bak output -1 -1 -3.2 -1.25`
  根据目视检查调整 corr_dx/corr_dy。

## 许可证

MIT
