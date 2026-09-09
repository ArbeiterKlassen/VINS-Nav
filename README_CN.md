# VINS-Nav: 伪双目视觉惯性 SLAM + 导航

> [English](README.md)

基于 Gazebo TurtleBot3 house 环境的端到端视觉惯性 SLAM 与导航系统。
核心创新是**伪双目技术**：通过单目 RGB-D 相机深度图生成虚拟右目图像，
使 OpenVINS 能够以双目模式运行，获得尺度正确的里程计输出。

导航由 **EGO-Planner**（浙大 FAST 实验室）负责，集成时未改动双方任何代码——
一个桥接节点把 OpenVINS 的里程计翻译成 EGO-Planner 期望的坐标系约定。
完整闭环：遥控建图 → RViz 点击目标 → 自主 B-spline 轨迹 → `/cmd_vel`。

**尺度精度：0.991x**（单目模式发散 200×）。  
**3.1m 轨迹漂移：2.8cm（0.9%）。**  
**方位误差：0.0°。**

---

## 技术博客

1. [伪双目 VIO + RTAB-Map 全链路 SLAM 系统搭建](https://soyorin.work/articles/000031.html)
2. [EGO-Planner 集成：VIO + 建图 + 规划全栈融合](https://soyorin.work/articles/000033.html)

---

## 目录

1. [系统架构](#系统架构)
2. [ROS 话题](#ros-话题)
3. [TF 树](#tf-树)
4. [环境与安装](#环境与安装)
5. [完整复现教程](#完整复现教程)
6. [EGO-Planner 集成](#ego-planner-集成)
7. [脚本说明](#脚本说明)
8. [配置参数](#配置参数)
9. [文件清单](#文件清单)
10. [性能指标](#性能指标)
11. [故障排除](#故障排除)

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

可选的自主导航（EGO-Planner），详见 §EGO-Planner 集成：
                          ┌────────────────┐
                          │   RTAB-Map     │
                          └───────┬────────┘
                                  │ /rtabmap/grid_map
                                  ▼
                          ┌────────────────┐   ┌────────────────┐
                          │ rtabmap_map_   │   │ vio_odom_      │
                          │ relay → /map   │   │ bridge         │
                          └───────┬────────┘   └───────┬────────┘
                                  │ /static_map        │ /odom_world
                                  ▼                    │
                          ┌────────────────┐           │
                          │  map_to_pc2    │           │
                          │  2D→3D 点云    │           │
                          └───────┬────────┘           │
                                  │ global_cloud       │
                                  ▼                    ▼
                          ┌────────────────────────────────────┐
                          │  ego_planner_node (A* + B-spline)  │
                          └───────────────┬────────────────────┘
                                          │ /planning/bspline
                                          ▼
                                  ┌────────────────┐
                                  │  traj_server   │
                                  └───────┬────────┘
                                          │ /cmd_vel
                                          ▼
                                       底盘
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

导航层 (AMCL + move_base，备选方案):
  map_server:  输入: (文件)  输出: /map (静态, 锁存)
  AMCL:        输入: /map, /scan  输出: /amcl_pose, /tf (map→odom)
  move_base:   输入: /map, /scan, /tf  输出: /cmd_vel

EGO-Planner 层 (可选，详见 §EGO-Planner 集成):
  vio_odom_bridge.py:
    输入:  /ov_msckf/odomimu
    输出:  /odom_world, TF odom→base_footprint

  rtabmap_map_relay.py:
    输入:  /rtabmap/grid_map
    输出:  /map, 服务 /static_map

  map_to_pc2 (来自 ego-planner):
    输入:  服务 /static_map
    输出:  /map_generator/global_cloud

  waypoint_generator:
    输入:  /move_base_simple/goal (RViz "2D Nav Goal"), /odom_world
    输出:  /waypoint_generator/waypoints

  ego_planner_node:
    输入:  /odom_world, /map_generator/global_cloud, /waypoint_generator/waypoints
    输出:  /planning/bspline

  traj_server:
    输入:  /planning/bspline, /odom_world
    输出:  /cmd_vel
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
| `/odom_world` | nav_msgs/Odometry | 50Hz | EGO-Planner 里程计桥接 (odom→base_footprint) |
| `/map` | nav_msgs/OccupancyGrid | 变化时 | 由 `/rtabmap/grid_map` 转发的实时地图（锁存） |
| `/map_generator/global_cloud` | sensor_msgs/PointCloud2 | 10Hz | 2D 栅格拉伸成的 3D 点云，供 EGO-Planner 建 ESDF |
| `/waypoint_generator/waypoints` | nav_msgs/Path | 收到目标时 | RViz "2D Nav Goal" 生成的导航目标 |
| `/planning/bspline` | ego_planner/Bspline | 重规划时 | 优化后的 B-spline 轨迹 |
| `/cmd_vel`（EGO 模式） | geometry_msgs/Twist | 20Hz | traj_server 的纯追踪速度指令 |

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

### EGO-Planner 模式下的 TF 树

EGO-Planner 期望 `odom` 是一个**固定的世界坐标系**，机器人在其中运动
（与原始 `fake_odom` 的语义一致）。此时坐标系链变为：

```
map ──(RTAB-Map mapData, 动态)──► odom ──(VIO 桥接, 动态)──► base_footprint ──(URDF)──► base_link
```

| 变换 | 发布者 | 类型 | 说明 |
|------|--------|------|------|
| `map → odom` | `map_tf_broadcaster.py` | 动态, 10Hz | RTAB-Map 从 `/rtabmap/mapData` 得到的定位修正 |
| `odom → base_footprint` | `vio_odom_bridge.py` | 动态, 50Hz | VIO 位姿，已应用 imu→base 的 Z 偏移 |
| `global → odom` | 静态发布器 | 静态恒等 | 必需，因为 RTAB-Map 的 `odom_frame_id=global` |
| `imu → base_footprint` | 静态发布器 | 静态 | 兜底，Z = −0.078 m |

**为什么还要保留静态 `odom → base_footprint` 兜底：** VIO 初始化完成前没有动态变换，
TF2 会报 *"Could not find a connection between 'global' and 'base_footprint'"*。
静态兜底保证坐标系树始终连通，等里程计到达后动态桥接自然接管。

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

### EGO-Planner 工作空间（仅 §EGO-Planner 集成 需要）

EGO-Planner 位于**第二个** catkin 工作空间，以 git submodule 形式引入：

```bash
git submodule update --init --recursive      # 在仓库根目录执行
cd ego-planner/planner
catkin_make                                  # 若已带预编译 devel/ 可跳过
```

两个工作空间需要叠加，且第二个必须用 *extend* 模式 source，否则会回滚第一个的环境：

```bash
source /home/nu/VINS-Nav/setup_ego_vio.sh    # 已处理叠加与 --extend
```

该脚本还会清理 EGO-Planner 工作空间注入 `ROS_PACKAGE_PATH` 的换行符——
不清理的话 `rospack` 会找不到任何包。

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

### 第四步：导航（AMCL + move_base）

```bash
# 使用预建地图启动导航栈
roslaunch ov_msckf nav_clean.launch

# 发送导航目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}" \
  "pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}"
```

### 第五步：建图 → 导航（EGO-Planner，单次会话完成）

```bash
# 终端 1：一键启动全部——Gazebo、VIO、RTAB-Map、EGO-Planner
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_rtabmap.launch

# 终端 2：遥控开车建图
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# 终端 3（可选）：观察地图增长
rostopic hz /map

# 地图满意后停车。
# 在 RViz 中按 G 键，点击目标点 → EGO-Planner 自动规划并驱动过去。
```

地图是实时的：RTAB-Map 的栅格被转发到 `/map` 和 `/static_map`，
`map_to_pc2` 每 2 秒重新读取一次，保证规划器的 ESDF 始终最新。

若改用**预建地图**（不含 RTAB-Map，无在线定位）：

```bash
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_vio.launch gazebo:=true map_file:=/path/to/map.yaml
```

### 第六步：回放录制的 bag（不用 Gazebo）

同一套栈也能直接跑在录制的 bag 上——VIO 在回放的相机/IMU 流上实时重跑，不需要仿真器：

```bash
# 终端 1：VIO + RTAB-Map + EGO-Planner，不启动 Gazebo
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_rtabmap.launch gazebo:=false rviz:=false

# 终端 2：只回放传感器话题和时钟
rosbag play ~/house_full.bag --clock \
  --topics /camera/rgb/image_raw /camera/depth/image_raw \
           /camera/rgb/camera_info /imu /clock
```

**绝对不要回放 `/tf` 和 `/tf_static`。** bag 里的 TF 是原始会话的里程计产生的，
会和 `vio_odom_bridge.py` 实时发布的 `odom → base_footprint` 在同一条边上冲突。

这样测试时有两个坑：

- 发导航目标要用**持续发布**的发布者。一次性 `rostopic pub -1` 可能在订阅连接建立
  之前就被丢弃，`waypoint_generator` 于是静默地永不触发。RViz 的 "2D Nav Goal" 没这个问题。
- 要预期 **VIO 漂移**。`pseudo_stereo` 是吞吐瓶颈（见 §性能指标），
  估计器拿到的立体帧数远低于所需。

---

## EGO-Planner 集成

[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)（浙大 FAST 实验室）是一个
基于梯度的局部轨迹规划器：先在欧几里得符号距离场（ESDF）上做 A* 搜索得到粗略路径，
再通过优化 B-spline 控制点、在平滑代价与碰撞代价之间取平衡，得到光滑无碰撞的
**B-spline 轨迹**。`traj_server` 用纯追踪控制把它转换成 `/cmd_vel`。

### 设计约束：双方代码均未修改

集成只新增了一个桥接层。OpenVINS 配置、`pseudo_stereo.py`、EGO-Planner 二进制、
`map_to_pc2.py` 和 `fake_odom.py` 都原封不动。唯一的例外是 `map_to_pc2.py` 里
**新增 6 行向后兼容代码**（`refresh_interval` 参数，默认 `0` 即原行为），
用于周期性重读实时更新的地图。

### 被桥接的差异

| | OpenVINS | EGO-Planner |
|---|----------|-------------|
| 里程计话题 | `/ov_msckf/odomimu` | `/odom_world` |
| `frame_id` | `global` | `odom` |
| `child_frame_id` | `imu` | `base_footprint` |
| 地图来源 | RTAB-Map `/rtabmap/grid_map` | `/map` + `/static_map` 服务 |

`vio_odom_bridge.py` 订阅 `/ov_msckf/odomimu`，应用固定的 IMU→base_footprint 偏移
（向下 0.078 m，按当前 IMU 姿态旋转），再以 `/odom_world` 重新发布，
同时以 50Hz 发布动态 TF `odom → base_footprint`。

`rtabmap_map_relay.py` 把 RTAB-Map 的 `/rtabmap/grid_map` 转发到 `/map`，
并提供一个 `/static_map` 服务——因为 RTAB-Map 本身不提供该服务，
而 `map_to_pc2` 启动时依赖它。

### 启动文件

| 启动文件 | 地图来源 | RTAB-Map | 用途 |
|----------|----------|----------|------|
| `turtlebot3_house_stereo.launch` | RTAB-Map | ✔ | 纯建图（原始） |
| `nav_ego_rtabmap.launch` | RTAB-Map，实时 | ✔ | **建图 → 导航，单次会话完成** |
| `nav_ego_vio.launch` | `map_server`（文件） | ✘ | 预建地图导航 |

### 已验证的话题契约

以下每条话题连接都经过核对：一是从编译好的 EGO-Planner 二进制符号表中提取
（发行版不含源码），二是通过实际启动测试确认：

```
waypoint_generator  订阅: /move_base_simple/goal, /odom_world   发布: /waypoint_generator/waypoints
ego_planner_node    订阅: /odom_world, /map_generator/global_cloud, /waypoint_generator/waypoints
                    发布: /planning/bspline  (ego_planner/Bspline)
traj_server         订阅: /planning/bspline, /odom_world         发布: /cmd_vel
map_to_pc2          订阅: 服务 /static_map                       发布: /map_generator/global_cloud
```

### 工作空间修复记录

EGO-Planner 工作空间是**预编译分发**的，不含源码。全新 clone 后先跑一次恢复脚本：

```bash
./scripts/setup_ego_workspace.sh        # 幂等，可重复运行
```

脚本会从**实际存在**的产物（编译好的 C++ 头文件和共享库）重新生成一切：

1. `devel/.catkin` 重指向本地源码目录。
2. 为 19 个包生成 `package.xml`（写入 `devel/share/`；`src/` 下若已存在真实
   `map_tools/package.xml` 则不覆盖）——否则 `rospack` 找不到任何包。
3. 从 C++ `Definition` 结构体重建 `*.msg` 定义，使 `rosmsg show` 及任何重新编译可用
   （发行版只带编译后的头文件）。
4. `devel/lib/*/` 下的 catkin 包装脚本从原构建机路径
   （`/home/bingoling/Desktop/planner/...`）重指向本地源码。

完整调试记录见 `docs/openvins_egoplanner.md`。

### 已知限制

`ego_planner_node` 的 FSM 会一直停在 `INIT`，直到 `/odom_world` 上有里程计数据。
没有 VIO 在跑（也没有 bag 回放）时它只是空转——这是预期行为，不是故障。
VIO 本身也需要静止几秒完成 ZUPT 初始化后才会发布数据。

---

## 脚本说明

### `pseudo_stereo.py` — 虚拟右目生成器

通过深度图和视差公式生成虚拟右目相机图像。

**输入话题:** `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`
**输出话题:** `/camera/right/image_raw`, `/camera/right/camera_info`
**算法:** 对每个有效深度像素 Z，计算视差 d = fx * 基线 / Z。右图像素列 (u − d) 的像素值取自左图像素列 u。逐行 Z-buffer 用 `np.lexsort((-disp, u_dst))` + 布尔分组起始 diff 实现（与早先的 `np.unique` 方案**逐像素相同**，但快约 25%）。剩余去遮挡空洞按 `~fill_mode` 填充。
**基线:** 0.08m（对标 Intel RealSense D435）。

**空洞填充（`~fill_mode`）** —— 室内深度下右目约 40% 是空洞，原先的全分辨率
Navier-Stokes inpainting 每帧约 170 ms，把节点压到 ~5 Hz，饿死了 VIO。每帧 640×480 实测：

| `fill_mode` | 耗时 | 频率 | 说明 |
|-------------|------|------|------|
| `morph`（默认） | 40.2 ms | 24.9 Hz | 形态学闭运算，把真实像素膨胀进空洞 |
| `none` | 32.8 ms | 30.5 Hz | 空洞保持黑色，对立体匹配最"诚实" |
| `inpaint_ds` | 49.3 ms | 20.3 Hz | 1/4 分辨率 TELEA inpaint 后上采样到空洞 |
| `inpaint_ns` | 201.5 ms | 5.0 Hz | 原始行为，保留以便对照 |

用默认值后，`/camera/right/image_raw` 从 4.4 Hz 提升到约 16 Hz（受输入速率限制）。
900 秒 bag 回放实测：

| 配置 | 结果 |
|------|------|
| 仅 VIO（伪双目 + OpenVINS） | 整段 900 秒 **NaN 为 0**；此前约 450 秒就发散 |
| 全栈（+ RTAB-Map + EGO-Planner） | 约 500 秒后仍出现 NaN（额外负载下伪双目降到约 13 Hz） |

也就是说：吞吐优化在只跑 VIO 时已消除瓶颈，但全栈还需要更多余量才能让建图保持干净——
详见 §故障排除。

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

**自动位姿跳变校正（新增）：** 录制的 VIO 轨迹存在单采样不连续点——
在 `house_full.bag` 上实测：一个 33ms 采样内位移 0.4m（12 m/s，而机器人物理上限只有
0.26 m/s），聚集成簇且全部落在 `x > 1.5` 区域——正是手工 `corr_dx`/`corr_dy`
补偿针对的那一块。`build_map.py` 现在会检测任何 `|dp|/dt` 超过 `v_max`（默认 1.0 m/s）
的采样，累加偏移并应用到其后的所有位姿，使轨迹保持连续。
设 `BUILD_MAP_NO_JUMP_FIX=1` 可复现旧行为。

> **如实说明：** 校正已实现并通过单元测试，但在 `house_full.bag` 上
> **没有带来可见的地图改善**（485×292 对比 462×292，障碍格 16,671 对比 16,758）。
> 主导误差在别处——最可能是**尺度膨胀**：同一条轨迹在正常运动段隐含 0.5–0.7 m/s，
> 是机器人物理上限的 2–3 倍。

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

### `vio_odom_bridge.py` — VIO → EGO-Planner 里程计桥接

把 OpenVINS 的里程计翻译成 EGO-Planner 期望的坐标系约定。

**输入:** `/ov_msckf/odomimu` (`global → imu`)
**输出:** `/odom_world` (`odom → base_footprint`)，50Hz，同时发布动态 TF `odom → base_footprint`

**变换:** `p_base = p_imu + R(imu) · (0, 0, −0.078)`——IMU 在 `base_footprint`
上方 0.078 m，把该固定偏移按 IMU 姿态旋转到世界系后叠加。姿态直接透传
（IMU 与 base_footprint 同姿态）。位姿/速度协方差一并转发，
方便下游判断定位质量。

必须用 `tf2_ros.TransformBroadcaster`——直接往 `/tf` 发裸 `TransformStamped`
**不会**构成合法的 `tf2_msgs/TFMessage`，TF2 会静默丢弃。

### `rtabmap_map_relay.py` — 实时地图转发

把 RTAB-Map 的实时占据栅格桥接到 EGO-Planner 需要的接口。

**输入:** `/rtabmap/grid_map`
**输出:** `/map`（锁存）+ 服务 `/static_map`

RTAB-Map 只发布 `grid_map`，既不发布 `/map` 也不提供 `/static_map` 服务，
而 `map_to_pc2` 启动时就要调用 `/static_map`。没有这个转发节点，
规划器看到的就是一个空世界。

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
    │   ├── nav_ego_rtabmap.launch            # EGO-Planner + RTAB-Map 实时地图
    │   ├── nav_ego_vio.launch                # EGO-Planner + 预建地图
    │   ├── nav_ego.rviz                      # RViz 配置（Fixed Frame: map）
    │   └── explore.launch                    # 前沿探索
    └── scripts/                      # 新增脚本
        ├── pseudo_stereo.py          # 深度→右目相机投影
        ├── dyn_odom_tf.py            # 动态 global→odom TF
        ├── odom_tf_pub.py            # 里程计→TF 桥接
        ├── frontier_explore.py       # cv2 前沿检测
        ├── amcl_tf_pub.py            # AMCL→TF 桥接
        ├── map_tf_broadcaster.py     # RTAB-Map→TF 桥接
        ├── vio_odom_bridge.py        # VIO→EGO-Planner 里程计桥接
        └── rtabmap_map_relay.py      # RTAB-Map 栅格→/map + /static_map
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
├── vio_odom_bridge.py    # VIO→EGO-Planner 里程计 + TF 桥接
├── rtabmap_map_relay.py  # RTAB-Map 栅格→/map + /static_map
└── tf_debug.py           # TF2 缓存诊断工具

setup_ego_vio.sh          # catkin_ws_ov + ego-planner 联合环境
ego-planner/              # git submodule（浙大 FAST 实验室 EGO-Planner）
docs/openvins_egoplanner.md  # 完整集成报告（原理 + 调试记录）
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

**伪双目吞吐是整个系统的瓶颈**（2026-09-09 实测：只跑 VIO 的隔离回放，排除 CPU 争抢）：

| 阶段 | 每帧 640×480 耗时 |
|------|-------------------|
| 逐行前向 warp | 62.9 ms |
| `cv2.inpaint(INPAINT_NS)` | **223.0 ms** |
| **合计** | **285.9 ms → 上限 3.5 Hz** |

录制的深度图只有 54.8% 有效像素，掩膜很大，Navier-Stokes inpainting 因此占绝对主导。
VIO 需要约 28 Hz 的立体帧，实际只有 3.5 Hz，导致跟踪退化、漂移累积，
最终输出 NaN 位姿。这正是「地图分离、需要手动 `corr_dx`/`corr_dy`」的根因。

EGO-Planner 栈（启动测试数据；完整闭环导航效果取决于 VIO 质量）：

| 指标 | 数值 | 备注 |
|------|------|------|
| 启动节点数 | 19/19 | Gazebo + VIO + RTAB-Map + 规划器 |
| 里程计桥接频率 | 50 Hz | `/odom_world` |
| 地图刷新周期 | 2 s | `map_to_pc2` 重读实时 RTAB-Map 栅格 |
| 加兜底后的 TF 错误 | 0 | 加静态 `odom→base_footprint` 前为 100+ |
| 规划视界 | 5.0 m / 5.0 s | EGO-Planner FSM |
| 最大线速度/角速度 | 0.5 m/s / 1.0 rad/s | `traj_server` 限制 |
| 到点容差 | 0.25 m | 纯追踪停止半径 |

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

### `rospack` 找不到任何 EGO-Planner 包
- 原因：预编译工作空间的 `devel/share/*/` 缺少 `package.xml`。
- 解决：见 §EGO-Planner 集成 → *工作空间修复记录*，并使用 `setup_ego_vio.sh`
  而不是手动逐个 source。

### `ROS_PACKAGE_PATH` 看起来正常，但 `rospack` 仍然失败
- 原因：EGO-Planner 工作空间往 `ROS_PACKAGE_PATH` 里注入了换行符，破坏了路径解析。
- 解决：`export ROS_PACKAGE_PATH=$(echo "$ROS_PACKAGE_PATH" | tr -d '\n\r')`
  （`setup_ego_vio.sh` 已自动处理）。

### source 第二个工作空间后第一个的环境失效
- 原因：catkin 的 `setup.bash` 默认会回滚先前 source 的工作空间。
- 解决：`CATKIN_SETUP_UTIL_ARGS="--extend" source <第二个工作空间>/devel/setup.bash`。

### TF 报错 *"Could not find a connection between 'global' and 'base_footprint'"*
- 原因：VIO 初始化前动态 `odom → base_footprint` 尚不存在，坐标系树断成两半。
- 解决：launch 文件中保留静态 `odom → base_footprint` 兜底；里程计到达后动态桥接会接管。

### 桥接节点发布了 TF，但 TF2 始终收不到
- 原因：往 `/tf` 发的是裸 `geometry_msgs/TransformStamped`，而该话题承载的是
  `tf2_msgs/TFMessage`，消息会被丢弃。
- 解决：使用 `tf2_ros.TransformBroadcaster().sendTransform()`。

### `ego_planner_node` 一直停在 `INIT`
- 原因：它在等 `/odom_world` 上的里程计；没有 VIO 在跑自然没有数据。干跑测试时同样如此。
- 解决：启动 VIO（或回放 bag），确认 `rostopic hz /odom_world` 非零。
  VIO 本身需要先静止几秒完成 ZUPT 初始化。

### 桥接/转发节点打印了 "ready"，但话题里没有数据
- 原因：节点在 `__init__` 中、注册到 master **之后**崩溃了。典型例子：
  `NameError: name 'tf2_ros' is not defined`——进程几毫秒内就退出，
  `rosnode list` 会短暂显示它，启动日志看起来也正常。
- 解决：一律用 `rostopic hz <话题>` 验证，不要用「节点起来了」当证据。
  到 `~/.ros/log/<本次运行>/` 下看该节点自己的日志找 traceback。
  另外注意 `rospy.Timer.run()` **不会**捕获回调异常——定时器回调里抛错会静默杀死定时器线程。

### `/map_tf_broadcaster` 因 `ROSTimeMovedBackwardsException` 退出
- 原因：`/clock` 跳变（bag 回放开始/结束）时 `rospy.Rate.sleep()` 会抛异常。
- 解决：捕获 `ROSTimeMovedBackwardsException` 并基于新时钟重建 `rospy.Rate`
  （`map_tf_broadcaster.py` 中已处理）。

### `waypoint_generator` 收到了目标却从不发布 waypoint
- 原因：目标是用一次性发布器（`rostopic pub -1`）发的，消息可能在订阅连接建立前
  就被丢弃，回调根本没触发。
- 解决：让发布者保持存活（RViz 的 "2D Nav Goal" 天然如此），或连续发布几秒。
  用 `rosnode info /waypoint_generator` 确认连接已建立。

### TF 警告 `TF_DENORMALIZED_QUATERNION ... (nan nan nan nan)`
- 原因：VIO 丢失跟踪，正在输出 NaN 位姿——通常是因为 `pseudo_stereo`
  饿死了它（见 §性能指标）。
- 解决：先查 `/camera/right/image_raw` 的频率；如果是几 Hz 而不是 ~28 Hz，
  先去修伪双目瓶颈。

## 许可证

MIT
