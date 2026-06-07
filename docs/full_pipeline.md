# Gazebo + Pseudo-Stereo OpenVINS + RTAB-Map 全链路 SLAM 系统

> 在 Gazebo TurtleBot3 仿真环境中，使用深度图生成虚拟双目图像，驱动 OpenVINS 实现精确 VIO，再通过 RTAB-Map 完成 RGBD SLAM 建图与 ROS Navigation 导航。

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulation                        │
│  TurtleBot3 House World                                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │ RGB Cam  │  │ Depth Cam│  │  IMU     │  │ Laser Scanner    │ │
│  │ 640x480  │  │ 640x480  │  │ 333Hz    │  │ 5Hz              │ │
│  │ 28Hz     │  │ 28Hz     │  │          │  │                  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘ │
└───────┼──────────────┼────────────┼──────────────────┼───────────┘
        │              │            │                  │
        ▼              ▼            ▼                  │
┌───────────────┐      │   ┌────────────────┐         │
│ Pseudo-Stereo │◄─────┘   │   OpenVINS     │         │
│ (depth warp)  │──right──►│  Stereo VIO    │         │
│ left=/rgb     │  image   │  max_cameras=2 │         │
│ right=/right  │          │  ZUPT init     │         │
└───────────────┘          └───────┬────────┘         │
                                   │ /odomimu          │
                                   ▼                   │
                          ┌────────────────┐           │
                          │   RTAB-Map     │◄──────────┘
                          │  RGBD SLAM     │  /scan
                          │  odom=VIO      │
                          └───────┬────────┘
                                  │ /grid_map, /cloud_map
                                  ▼
                          ┌────────────────┐
                          │   Navigation   │
                          │ map_server     │
                          │ AMCL           │
                          │ move_base      │
                          └────────────────┘
```

---

## 2. Gazebo 仿真环境

### 机器人模型
自定义 `turtlebot3_waffle_openvins.urdf.xacro`，基于标准 TB3 Waffle：

- RGBD 相机（640×480, 28Hz, fx=402.3）
- IMU 传感器（333Hz actual, gaussianNoise=0.015）
- 激光扫描（360°, 5Hz）
- 差速轮式里程计

### 关键修复
| 问题 | 修复 |
|------|------|
| IMU gaussianNoise=0.0 → 静态初始化永远失败 | 设为 0.015 |
| Gazebo publishOdomTF 导致 TF 双父帧冲突 | 设为 false |
| Kalibr noise_density 不匹配 | Allan 方差实测后精确校准为 0.00083 |

---

## 3. Pseudo-Stereo：深度转双目

### 原理
OpenVINS 不支持深度图输入（仅 Mono/Stereo）。利用深度图 + stereo disparity 公式生成虚拟右目：

```
disparity = fx × baseline / Z
u_right = u_left - disparity
```

### 实现 (`scripts/pseudo_stereo.py`)
- 订阅 `/camera/rgb/image_raw` + `/camera/depth/image_raw`
- 逐行计算 disparity，np.unique 做 Z-buffer（最近点优先）
- INPAINT_NS 填充空洞
- 发布 `/camera/right/image_raw` + camera_info

### 参数
- baseline: 0.08m（与 RealSense D435 相似）
- warp 速度：向量化，640×480 单帧 <10ms

---

## 4. OpenVINS Stereo VIO

### 配置 (`config/rgbd_stereo/`)
```
max_cameras: 2
use_stereo: true
try_zupt: true           # 零速更新，静态初始化
init_imu_thresh: 0.05    # 匹配实测 IMU 方差
init_max_features: 200
num_pts: 600
calib_cam_intrinsics: false
calib_cam_timeoffset: false
```

### IMU 噪声校准
录 54 秒静止 IMU bag，Allan 方差分析：
```
accelerometer_noise_density: 0.00083 m/s²/√Hz
gyroscope_noise_density:      0.00083 rad/s/√Hz
accelerometer_random_walk:    1e-6    (仿真无真实 drift)
gyroscope_random_walk:        1e-7
```

### 性能
| 指标 | 值 |
|------|-----|
| 初始化方式 | ZUPT 静态初始化 |
| 初始化时间 | < 1ms |
| 跟踪特征数 | 190-276 |
| 方位误差 | 0.0° |
| 尺度比率 | 0.991x |
| 3.1m 行程漂移 | 2.8cm (0.9%) |

> 对比：单目 VIO 在相同场景下尺度发散至 **200x** 以上。Pseudo-stereo 将尺度精度提升了 **200 倍**。

---

## 5. RTAB-Map RGBD SLAM

### 配置
```
odom_frame_id: global       # 匹配 OV odometry 的 frame_id
subscribe_depth: true
approx_sync: true
RGBD/NeighborLinkRefining: true
RGBD/ProximityBySpace: true
Reg/Strategy: 0              # Visual
```

### TF 树
```
map ──→ global ──→ base_footprint ──→ base_link ──→ camera_link ──→ ...
        ↑               ↑
   RTAB-Map        static TF bridge
                   (OV odometry frame)
```

修复了两个关键 TF 问题：
1. OV 自带 TF 发布器 333Hz 洪水 → `publish_global_to_imu_tf:=false`，改用 odom_tf_pub @30Hz
2. base_footprint 双父帧冲突 → 禁用 Gazebo publishOdomTF

---

## 6. Navigation Stack

### 组件
- `map_server`：服务 RTAB-Map 导出的 occupancy grid map
- `AMCL`：基于激光扫描的粒子滤波定位
- `move_base`：Navfn 全局规划 + DWA 局部规划

### 启动
```bash
roslaunch ov_msckf nav_clean.launch
```

### Frontier 自动探索
`scripts/frontier_explore.py`：
1. 读取 `/rtabmap/grid_map`
2. 检测 FREE-UNKNOWN 边界（frontier cells）
3. DBSCAN 聚类，按大小和距离评分
4. 发 `/move_base_simple/goal` 驱动机器人

---

## 7. 启动流程

```bash
# 1. 全系统（Gazebo + Pseudo-Stereo + OpenVINS + RTAB-Map）
roslaunch ov_msckf turtlebot3_house_stereo.launch

# 2. 驱动机器人探索建图（二选一）
~/catkin_ws_ov/scripts/auto_explore.py          # 固定路径
rosrun ov_msckf frontier_explore.py              # 自动前沿探索

# 3. 导出地图
rosrun map_server map_saver map:=/rtabmap/grid_map -f ~/catkin_ws_ov/maps/house_vio

# 4. 导航
roslaunch ov_msckf nav_clean.launch
```

---

## 8. 文件索引

```
~/catkin_ws_ov/
├── src/open_vins/ov_msckf/
│   ├── launch/
│   │   ├── turtlebot3_house_stereo.launch   # 主启动文件
│   │   ├── nav_clean.launch                  # 导航启动
│   │   └── navigation.launch                 # 旧导航文件
│   ├── scripts/
│   │   ├── pseudo_stereo.py                  # 深度→右目 warp
│   │   ├── odom_tf_pub.py                    # OV odom→TF bridge
│   │   ├── amcl_tf_pub.py                    # AMCL pose→TF
│   │   ├── frontier_explore.py               # 前沿自动探索
│   │   ├── auto_explore.py                   # 固定路径探索
│   │   ├── analyze_imu_noise.py              # Allan 方差分析
│   │   └── map_tf_broadcaster.py             # RTAB-Map TF 桥接
│   └── gazebo_models/urdf/
│       ├── turtlebot3_waffle_openvins.urdf.xacro
│       └── turtlebot3_waffle_openvins.gazebo.xacro
├── config/
│   ├── rgbd_stereo/        # Pseudo-Stereo VIO 配置
│   └── nav/                # 导航参数
├── maps/                   # 导出地图
└── docs/
    └── full_pipeline.md    # 本文档
```

---

## 9. 未来工作

### EGO Planner 接入
```
OpenVINS odom ──→ 位姿估计
Depth map ──→ ESDF ──→ EGO Planner ──→ 轨迹/cmd_vel
Grid map ──→ A* 全局路径 ──→ 引导 EGO Planner
```

### 墙面补全
用户已有 LineXT 点云补全工程，可对接 RTAB-Map 的 `/cloud_map` 做后处理。

### 动态障碍物
接入 `/scan` 做动态障碍物检测 + EGO Planner 实时避障。
