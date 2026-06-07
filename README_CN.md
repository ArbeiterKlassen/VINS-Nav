# VINS-Nav: 伪双目视觉惯性 SLAM + 导航

> [English](README.md)

基于 Gazebo TurtleBot3 house 环境，使用 OpenVINS 双目 VIO + RTAB-Map RGBD SLAM + ROS Navigation Stack。核心创新是通过深度图生成虚拟右目的伪双目技术，使单目 RGB-D 相机获得尺度正确的双目视觉惯性里程计。

**尺度精度：0.991x**（单目模式发散 200×）。  
**3.1m 轨迹漂移：2.8cm（0.9%）。**  
**方位误差：0.0°。**

## 系统架构

```
Gazebo TurtleBot3 House (12m × 12m)
├── RGB-D 相机 (640×480, 28Hz)
├── IMU (333Hz, gaussianNoise=0.015)
└── 激光扫描 (360°, 5Hz)

数据流：
  RGB + Depth → pseudo_stereo.py → 虚拟右目 (视差投影)
  左目 + 右目 + IMU → OpenVINS (双目, max_cameras=2) → /ov_msckf/odomimu
  RGB + Depth + 里程计 → RTAB-Map → /rtabmap/grid_map
  栅格地图 → map_server + AMCL + move_base → 导航

TF 链：
  map → global → odom → base_footprint → base_link → sensors
```

## 环境要求

- Ubuntu 20.04, ROS Noetic
- TurtleBot3: `ros-noetic-turtlebot3-gazebo`, `ros-noetic-turtlebot3-teleop`
- RTAB-Map: `ros-noetic-rtabmap-ros`
- 导航: `ros-noetic-move-base`, `ros-noetic-amcl`, `ros-noetic-map-server`
- OpenVINS: catkin workspace 中编译
- Python: numpy, opencv-python, pyyaml, rosbag

## 文件清单

### 对上游 OpenVINS 的修改

| 文件 | 说明 |
|------|------|
| `openvins/config/rgbd_stereo/` | 新增双目配置（估计器、IMU、相机链） |
| `openvins/ov_msckf/launch/turtlebot3_house_stereo.launch` | 主启动文件 |
| `openvins/ov_msckf/launch/rtabmap_db.launch` | RTAB-Map 节点（含 odom remap） |
| `openvins/ov_msckf/launch/nav_clean.launch` | 导航栈（AMCL + move_base） |
| `openvins/ov_msckf/launch/explore.launch` | 前沿探索 + move_base |
| `openvins/ov_msckf/launch/offline_bag.launch` | 离线 bag 处理 |
| `openvins/ov_msckf/launch/turtlebot3_house_openvins.launch` | 原始单目配置（保留） |

### 新增 OpenVINS 脚本

| 脚本 | 功能 |
|------|------|
| `ov_msckf/scripts/pseudo_stereo.py` | 深度→视差投影生成虚拟右目，基线 0.08m |
| `ov_msckf/scripts/dyn_odom_tf.py` | 从 VIO 里程计动态发布 global→odom TF |
| `ov_msckf/scripts/odom_tf_pub.py` | 里程计→TF 桥接（30Hz） |
| `ov_msckf/scripts/frontier_explore.py` | 基于 cv2 的前沿检测（无需 sklearn） |
| `ov_msckf/scripts/amcl_tf_pub.py` | AMCL 位姿→TF 发布器 |
| `ov_msckf/scripts/map_tf_broadcaster.py` | RTAB-Map 地图数据→TF 桥接 |

### 新增 Gazebo 模型

| 文件 | 说明 |
|------|------|
| `ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.urdf.xacro` | 机器人运动学链 |
| `ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.gazebo.xacro` | 传感器插件（gaussianNoise=0.015, publishOdomTF=false） |

### 导航配置

| 文件 | 说明 |
|------|------|
| `openvins/config/nav/costmap_common_params.yaml` | 公共 costmap（探测范围、足迹） |
| `openvins/config/nav/global_costmap_params.yaml` | 全局 costmap（静态, 30×30m） |
| `openvins/config/nav/local_costmap_params.yaml` | 局部 costmap（滑动窗口, 4×4m） |
| `openvins/config/nav/dwa_local_planner_params.yaml` | DWA 规划器（最大 0.22m/s） |
| `openvins/config/nav/navfn_global_planner_params.yaml` | Navfn 全局规划器 |

### 独立工具 (scripts/)

| 脚本 | 功能 |
|------|------|
| `build_map.py` | 离线建图：bag → OV 位姿 + 深度图 → 占用栅格 |
| `postprocess_v5.py` | 地图后处理：去噪、边界填充、门洞保护、漂移遮罩 |
| `postprocess_map.py` | 轻量地图后处理 |
| `analyze_imu_noise.py` | Allan 方差 IMU 噪声分析 |
| `offline_final.sh` | 一键离线 pipeline |
| `bounce_explore.py` | 激光反应式弹跳探索 |
| `laser_circle.py` | 右侧墙跟随循环探索 |
| `tf_debug.py` | TF2 缓存诊断 |

## 使用

### 在线（Gazebo 实时）

```bash
source ~/catkin_ws_ov/devel/setup.bash
export TURTLEBOT3_MODEL=waffle

# 启动全系统
roslaunch ov_msckf turtlebot3_house_stereo.launch

# 手动驾驶
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# 导出地图
rosrun map_server map_saver map:=/rtabmap/grid_map -f house_map
```

### 离线（bag 处理）

```bash
# 录制 bag（仅传感器，无需里程计）
rosbag record -O house.bak \
  /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info \
  /imu /clock /tf /tf_static

# 直接从 bag 建图
python3 scripts/build_map.py house.bak output_map

# 后处理
python3 scripts/postprocess_v5.py output_map.yaml output_map_final.pgm
```

### 导航

```bash
roslaunch ov_msckf nav_clean.launch
```

## 关键配置

`openvins/config/rgbd_stereo/estimator_config.yaml`:
- `try_zupt: true`, `init_imu_thresh: 0.05`
- `init_max_features: 200`, `num_pts: 600`
- `init_dyn_mle_max_iter: 0`（MLE 已禁用——仿真环境特征不足）

`openvins/config/rgbd_stereo/kalibr_imu_chain.yaml`:
- `noise_density: 0.00083`（Allan 方差实测）
- `random_walk: 1e-6 / 1e-7`

`openvins/config/rgbd_stereo/kalibr_imucam_chain.yaml`:
- `cam0: /camera/rgb/image_raw`, `cam1: /camera/right/image_raw`
- `baseline: 0.08m`

## 已知问题

1. **VIO 长时间漂移**：10 分钟以上轨迹的右半部分漂移约 3m。`build_map.py` 通过空间分割修正（`ox < -1.0`）。在线使用可通过 RTAB-Map 的图优化实现闭环校正。

2. **Gazebo 传感器衰减**：GUI 模式运行约 3 分钟后相机和 IMU 停止发布。解决方案：headless 模式或录制 bag 离线处理。

3. **门洞堵塞**：窄通道（<0.2m）可能在栅格地图中显示为实墙。`build_map.py` 的墙体检测偏向保守。

4. **RTAB-Map 离线模式缺陷**：odom 话题 remap 在离线 bag 回放时不会创建订阅。离线建图请使用 `build_map.py`。

## 性能

| 指标 | 数值 |
|------|------|
| 尺度比例 | 0.991x |
| 3.1m 漂移 | 2.8cm (0.9%) |
| 方位误差 | 0.0° |
| 初始化时间 | <1ms |
| 跟踪特征数 | 190–276 |
| 地图覆盖 | 398×317 (19.9m×15.9m) |
| 障碍物格数 | 14,512 |

## 团队

- VIO & 建图: [你的名字]
- 导航: [团队成员]
- EGO Planner: [团队成员]

## 许可证

MIT
