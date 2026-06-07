# VINS-Nav: Pseudo-Stereo Visual-Inertial SLAM + Navigation

> [中文](README_CN.md)

End-to-end visual-inertial SLAM and navigation on a simulated TurtleBot3 in Gazebo.
The core innovation is **pseudo-stereo**: a depth-to-disparity warping node that generates
a virtual right camera from a single RGB-D sensor, enabling OpenVINS to operate in
metric-scale stereo mode.

**Scale accuracy: 0.991x** (vs. 200× divergence in monocular mode).  
**3.1m trajectory drift: 2.8cm (0.9%).**  
**Orientation error: 0.0°.**

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [ROS Topics](#ros-topics)
3. [TF Tree](#tf-tree)
4. [Requirements & Installation](#requirements--installation)
5. [Step-by-Step Reproduction](#step-by-step-reproduction)
6. [Script Reference](#script-reference)
7. [Configuration Reference](#configuration-reference)
8. [File Inventory](#file-inventory)
9. [Performance](#performance)
10. [Troubleshooting](#troubleshooting)

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    Gazebo TurtleBot3 House (12m×12m)               │
│                                                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ RGB Cam  │  │ Depth Cam│  │   IMU    │  │  Laser Scanner   │  │
│  │ 640×480  │  │ 640×480  │  │  333Hz   │  │  360°, 5Hz       │  │
│  │  28Hz    │  │  28Hz    │  │          │  │                  │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
└───────┼──────────────┼────────────┼──────────────────┼───────────┘
        │              │            │                  │
        ▼              ▼            ▼                  │
 ┌──────────────┐      │    ┌────────────────┐         │
 │Pseudo-Stereo │◄─────┘    │   OpenVINS     │         │
 │ disparity    │──right──►│   Stereo VIO   │         │
 │ = fx*B/Z     │  image   │  max_cameras=2 │         │
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
                          │  Navigation    │
                          │  map_server    │
                          │  AMCL          │
                          │  move_base     │
                          └────────────────┘
```

### Data Flow

```
Sensor Layer (Gazebo):
  /camera/rgb/image_raw       [sensor_msgs/Image]      640×480, ~28Hz
  /camera/depth/image_raw     [sensor_msgs/Image]      640×480, ~28Hz
  /camera/rgb/camera_info     [sensor_msgs/CameraInfo]  fx=402.3, cx=320.5
  /imu                        [sensor_msgs/Imu]         ~333Hz
  /scan                       [sensor_msgs/LaserScan]  360°, 5Hz, 0.12-3.5m
  /clock                      [rosgraph_msgs/Clock]    sim time

Processing Layer:
  pseudo_stereo.py:
    IN:  /camera/rgb/image_raw, /camera/depth/image_raw, /camera/rgb/camera_info
    OUT: /camera/right/image_raw, /camera/right/camera_info

  OpenVINS (run_subscribe_msckf):
    IN:  /camera/rgb/image_raw, /camera/right/image_raw, /imu
    OUT: /ov_msckf/odomimu, /ov_msckf/poseimu, /ov_msckf/pathimu

  dyn_odom_tf.py:
    IN:  /ov_msckf/odomimu
    OUT: /tf (global→odom)

  RTAB-Map:
    IN:  /camera/rgb/image_raw, /camera/depth/image_raw,
         /camera/rgb/camera_info, /ov_msckf/odomimu
    OUT: /rtabmap/grid_map, /rtabmap/cloud_map, /rtabmap/info,
         /rtabmap/mapData, /rtabmap/mapGraph

Navigation Layer:
  map_server:  IN: (file)  OUT: /map (static, latched)
  AMCL:        IN: /map, /scan  OUT: /amcl_pose, /tf (map→odom)
  move_base:   IN: /map, /scan, /tf  OUT: /cmd_vel
```

---

## ROS Topics

### Published by This System

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/camera/right/image_raw` | sensor_msgs/Image | ~28Hz | Virtual right camera from depth warping |
| `/camera/right/camera_info` | sensor_msgs/CameraInfo | ~28Hz | Intrinsics for right camera (adjusted P matrix) |
| `/ov_msckf/odomimu` | nav_msgs/Odometry | ~300Hz | VIO odometry (frame: global, child: imu) |
| `/ov_msckf/poseimu` | geometry_msgs/PoseWithCovarianceStamped | ~28Hz | Pose estimate with covariance |
| `/ov_msckf/pathimu` | nav_msgs/Path | ~28Hz | Trajectory history |
| `/rtabmap/grid_map` | nav_msgs/OccupancyGrid | on change | 2D occupancy grid (global) |
| `/rtabmap/cloud_map` | sensor_msgs/PointCloud2 | on change | 3D point cloud map |
| `/rtabmap/info` | rtabmap_msgs/Info | ~1Hz | Map statistics (refId, loopClosureId) |
| `/rtabmap/mapData` | rtabmap_msgs/MapData | on change | Full map graph with mapToOdom |
| `/rtabmap/mapGraph` | rtabmap_msgs/MapGraph | on change | Pose graph visualization |
| `/tf` (global→odom) | tf2_msgs/TFMessage | 30Hz | Dynamic VIO pose from dyn_odom_tf.py |

### Subscribed Topics (from Gazebo)

| Topic | Type | Rate | Consumer |
|-------|------|------|----------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | ~28Hz | pseudo_stereo, OpenVINS, RTAB-Map |
| `/camera/depth/image_raw` | sensor_msgs/Image | ~28Hz | pseudo_stereo, RTAB-Map |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | ~28Hz | pseudo_stereo, RTAB-Map |
| `/imu` | sensor_msgs/Imu | ~333Hz | OpenVINS |
| `/scan` | sensor_msgs/LaserScan | ~5Hz | AMCL, move_base |
| `/clock` | rosgraph_msgs/Clock | ~100Hz | All nodes (use_sim_time=true) |

---

## TF Tree

```
                           ┌─────────────────┐
                           │       map       │  (RTAB-Map SLAM result)
                           └────────┬────────┘
                                    │
                           ┌────────▼────────┐
                           │     global      │  (OpenVINS world frame)
                           └────────┬────────┘
                                    │ dyn_odom_tf.py (30Hz, from /ov_msckf/odomimu)
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

**Key design decisions:**
- `global→odom` is DYNAMIC (from VIO odometry). This is essential — a static TF here prevents RTAB-Map from tracking robot motion.
- `odom→base_footprint` is static (0,0,-0.078). The IMU is 0.078m above the base footprint.
- `base_footprint` must have ONLY ONE parent. Never publish both `imu→base_footprint` and `odom→base_footprint` simultaneously.
- OpenVINS's internal TF publisher (`publish_global_to_imu_tf`) is disabled to avoid 333Hz TF buffer flooding.

---

## Requirements & Installation

### System
- Ubuntu 20.04
- ROS Noetic (full desktop)
- Python 3.8+

### ROS Packages
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

### Python Dependencies
```bash
pip3 install numpy opencv-python pyyaml
```

### OpenVINS Compilation
```bash
mkdir -p ~/catkin_ws_ov/src
cd ~/catkin_ws_ov/src
git clone https://github.com/rpng/open_vins.git
cd ~/catkin_ws_ov
catkin_make -j4
source devel/setup.bash
```

### VINS-Nav Files
Copy `openvins/` contents into `~/catkin_ws_ov/src/open_vins/`, overwriting as needed.
The standalone scripts in `scripts/` can run anywhere.

```bash
# After copying:
cd ~/catkin_ws_ov
catkin_make  # recompile if launch files are new
```

---

## Step-by-Step Reproduction

### 1. Online: Live Gazebo Mapping

```bash
# Terminal 1: Launch everything
source ~/catkin_ws_ov/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch ov_msckf turtlebot3_house_stereo.launch

# Wait for "successful initialization" in OpenVINS output (~45s)

# Terminal 2: Drive the robot
export TURTLEBOT3_MODEL=waffle
source ~/catkin_ws_ov/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Terminal 3: Export map when done
rosrun map_server map_saver map:=/rtabmap/grid_map -f ~/maps/house_live
```

### 2. Offline: Bag Recording + Processing

```bash
# ---- Recording (do this once) ----
# Terminal 1: Launch system
roslaunch ov_msckf turtlebot3_house_stereo.launch gui:=true

# Terminal 2: Record bag
rosbag record -O house.bak \
  /camera/rgb/image_raw \
  /camera/depth/image_raw \
  /camera/rgb/camera_info \
  /imu \
  /clock \
  /tf \
  /tf_static

# Terminal 3: Drive
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# Drive through the entire house, then Ctrl-C the bag recording

# ---- Processing (repeatable) ----
# Build map directly from bag (no RTAB-Map needed)
python3 scripts/build_map.py house.bak output_map

# Post-process
python3 scripts/postprocess_v5.py output_map.yaml output_map_final.pgm
```

### 3. Navigation

```bash
# Launch navigation stack with a pre-built map
roslaunch ov_msckf nav_clean.launch

# Send a goal
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}" \
  "pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}"
```

---

## Script Reference

### `pseudo_stereo.py` — Virtual Stereo Generator

Generates a virtual right camera image from RGB + depth using the stereo disparity formula.

**Input topics:** `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`
**Output topics:** `/camera/right/image_raw`, `/camera/right/camera_info`
**Algorithm:** For each depth pixel with value Z, compute disparity d = fx * baseline / Z. The right-image pixel at column (u - d) gets the left-image pixel value from column u. Z-buffer via numpy's `unique` ensures occluded surfaces are hidden. Navier-Stokes inpainting fills disocclusion holes.
**Baseline:** 0.08m (matching Intel RealSense D435).
**Performance:** <10ms per 640×480 frame.

### `dyn_odom_tf.py` — Dynamic TF Bridge

Publishes the VIO odometry pose as a dynamic TF transform `global→odom` at 30Hz.
This is essential for RTAB-Map to track robot motion — a static TF would lock the robot
in place.

**Input:** `/ov_msckf/odomimu` (nav_msgs/Odometry)
**Output:** `/tf` (global→odom at 30Hz)

### `odom_tf_pub.py` — Alternative TF Bridge

Same purpose as dyn_odom_tf.py but reads frame_id and child_frame_id from the
odometry message directly. Used in the online pipeline.

### `build_map.py` — Offline Map Builder

Processes a ROS bag directly to build a 2D occupancy grid, completely bypassing RTAB-Map.

**Input:** ROS bag file containing `/tf`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`
**Output:** PGM + YAML occupancy grid map
**Algorithm:**
1. Extract robot poses from bag TF (`global→imu`)
2. For each depth frame, find nearest TF pose (within 100ms tolerance)
3. Project depth pixels to 3D in camera frame: Xc = (u-cx)*Z/fx, Yc = (v-cy)*Z/fy
4. Rotate to IMU frame: Xi=Zc, Yi=-Xc, Zi=-Yc
5. Transform to world frame using IMU→global quaternion
6. Accumulate points, classify as wall (0.15m < Z < 3.0m) or floor (-0.5m < Z < 0.15m)
7. Grid: occ = wall-dominant cells (gw*3 > gf), free = clean floor (gf>20, gw==0)
8. VIO drift correction: apply spatial offset to left-half poses (ox < -1.0)

**Arguments:** `build_map.py <bag_path> [output_prefix] [max_time] [min_time] [corr_dx] [corr_dy]`
**Example:** `build_map.py house.bak map -1 -1 -3.2 -1.25` (full bag, drift correction)

### `postprocess_v5.py` — Map Post-Processor

Cleans and completes the occupancy grid.

**Processing steps:**
1. Denoise: remove occupied cells with <3 neighbors (depth sensor noise)
2. Drift filter: remove horizontally elongated (>3:1 aspect ratio) or small (<100px) clusters
3. Boundary fill: fill unknown cells adjacent to free space as occupied (wall completion)
4. Narrow gap protection: skip filling cells where free space exists 1-4px away on both sides (doorways)
5. Manual drift mask: user-identified regions for surgical noise removal

### `analyze_imu_noise.py` — IMU Noise Calibrator

Performs Allan variance analysis on a recorded IMU bag to extract noise density
and random walk parameters for the Kalibr config.

**Usage:** `analyze_imu_noise.py <imu_bag>`

### `frontier_explore.py` — Autonomous Explorer

Detects frontier cells (free space adjacent to unknown) in the occupancy grid,
clusters them using cv2.connectedComponents, and sends goals to move_base.

**No sklearn dependency** — uses only OpenCV.
**Input:** `/rtabmap/grid_map`, `/move_base/status`
**Output:** `/move_base_simple/goal`

### `bounce_explore.py` — Reactive Explorer

Laser-based reactive exploration: drives straight until blocked, finds largest
angular gap in scan, turns toward it. Narrow-space detection prevents oscillation.
No map or planning required.

### `laser_circle.py` — Wall-Following Explorer

Right-side wall-following behavior using laser scan. Maintains ~0.6m distance from
the right wall. Useful for systematic coverage of rooms.

---

## Configuration Reference

### `estimator_config.yaml`

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `try_zupt` | true | Zero-velocity update enables static initialization |
| `init_imu_thresh` | 0.05 | Above measured IMU excitation variance (~0.026) |
| `init_max_features` | 200 | More features for better initialization quality |
| `num_pts` | 600 | High feature count for tracking stability |
| `use_stereo` | true | Enable stereo feature constraints |
| `max_cameras` | 2 | Left (real) + Right (virtual) |
| `init_dyn_mle_max_iter` | 0 | MLE disabled — insufficient features in simulation |
| `calib_cam_intrinsics` | false | Fixed intrinsics (known from Gazebo) |
| `calib_cam_timeoffset` | false | No time offset in simulation |
| `calib_cam_extrinsics` | false | Fixed extrinsics (verified from URDF) |

### `kalibr_imu_chain.yaml`

| Parameter | Value | Source |
|-----------|-------|--------|
| `accelerometer_noise_density` | 0.00083 | Allan variance of 54s stationary IMU data |
| `gyroscope_noise_density` | 0.00083 | Same analysis |
| `accelerometer_random_walk` | 1e-6 | Simulation has no real bias drift |
| `gyroscope_random_walk` | 1e-7 | Same |
| `update_rate` | 400 | Nominal (actual ~333Hz) |

### `kalibr_imucam_chain.yaml`

| Parameter | cam0 (left) | cam1 (right) |
|-----------|-------------|--------------|
| `rostopic` | `/camera/rgb/image_raw` | `/camera/right/image_raw` |
| `resolution` | [640, 480] | [640, 480] |
| `intrinsics` | [403, 403, 320, 240] | [403, 403, 320, 240] |
| `distortion` | radtan, all zeros | radtan, all zeros |
| `T_cam_imu` translation | [-0.047, 0.039, -0.069] | [-0.127, 0.039, -0.069] |

The 0.08m baseline between cam0 and cam1 is encoded in the translation difference: -0.047 - (-0.127) = 0.08.

### Gazebo Model Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `gaussianNoise` (IMU) | 0.015 | Matches noise_density=0.00083 at 400Hz |
| `publishOdomTF` | false | Avoids dual-parent conflict on base_footprint |
| `updateRateHZ` (IMU) | 400 | Maximum rate for good IMU preintegration |
| `horizontal_fov` (camera) | 1.3439 rad | Standard 640×480 FOV |

---

## File Inventory

### Modified from Upstream OpenVINS

```
openvins/
├── config/
│   ├── nav/                          # Navigation costmap & planner params (new)
│   └── rgbd_stereo/                  # Stereo VIO config (new)
│       ├── estimator_config.yaml     # ZUPT init, 600 features, MLE disabled
│       ├── kalibr_imu_chain.yaml     # Allan-calibrated noise params
│       └── kalibr_imucam_chain.yaml  # 2-camera stereo rig, B=0.08m
└── ov_msckf/
    ├── gazebo_models/urdf/           # Custom TurtleBot3 model (new)
    │   ├── turtlebot3_waffle_openvins.urdf.xacro
    │   └── turtlebot3_waffle_openvins.gazebo.xacro
    ├── launch/                       # Launch files (new)
    │   ├── turtlebot3_house_stereo.launch    # Main: Gazebo + stereo OV + RTAB-Map
    │   ├── turtlebot3_house_openvins.launch  # Original monocular (preserved)
    │   ├── rtabmap_db.launch                 # RTAB-Map solo with odom remap
    │   ├── offline_bag.launch                # Offline processing pipeline
    │   ├── nav_clean.launch                  # AMCL + move_base
    │   └── explore.launch                    # Frontier exploration
    └── scripts/                      # New scripts
        ├── pseudo_stereo.py          # Depth→right camera warping
        ├── dyn_odom_tf.py            # Dynamic global→odom TF
        ├── odom_tf_pub.py            # Odom→TF bridge
        ├── frontier_explore.py       # cv2 frontier detection
        ├── amcl_tf_pub.py            # AMCL→TF bridge
        └── map_tf_broadcaster.py     # RTAB-Map→TF bridge
```

### Standalone Tools

```
scripts/
├── build_map.py          # Offline bag→map pipeline
├── postprocess_v5.py     # Map post-processing (denoise, fill, doorway protect)
├── postprocess_map.py    # Lightweight map post-processing
├── analyze_imu_noise.py  # Allan variance IMU noise analysis
├── offline_final.sh      # One-shot offline processing script
├── bounce_explore.py     # Reactive laser-based exploration
├── laser_circle.py       # Wall-following exploration
└── tf_debug.py           # TF2 buffer diagnostic tool
```

---

## Performance

| Metric | Value | Notes |
|--------|-------|-------|
| Scale ratio | 0.991x | 3.1m trajectory |
| Drift | 2.8cm (0.9%) | Over 3.1m travel |
| Orientation error | 0.0° | 62° commanded turn |
| Init time | <1ms | ZUPT static initialization |
| Features tracked | 190–276 | After optimization |
| Map coverage | 398×317 (19.9m×15.9m) | Final post-processed |
| Occupied cells | 14,512 | Walls + furniture |
| Pseudo-stereo latency | <10ms | Per 640×480 frame |
| OV tracking rate | 100–300Hz | Varies with camera/IMU ratio |
| Bag→map processing | ~5 min | 35GB bag, 27M point cloud |

---

## Troubleshooting

### "failed static init: no accel jerk detected"
- Cause: IMU noise too low for static initialization jerk detection.
- Fix: Set `gaussianNoise` >= 0.01 in the Gazebo IMU plugin, and `init_imu_thresh` >= measured IMU variance.

### "failed initialization in 0.0000 seconds"
- Cause: OpenVINS feature database empty or `use_sim_time` not set.
- Fix: Verify `/clock` is publishing and `rosparam get /use_sim_time` returns true.

### RTAB-Map "Did not receive data since 5 seconds"
- Cause: Approximate sync cannot match camera topics.
- Fix: Check that `/camera/rgb/image_raw`, `/camera/depth/image_raw`, and `/camera/rgb/camera_info` all have similar timestamps (within 0.1s).

### Map stays at 118×88, never expands
- Cause: Static TF `global→base_footprint` overriding dynamic odometry.
- Fix: Replace static TF with `dyn_odom_tf.py` (dynamic `global→odom`) + static `odom→base_footprint`.

### Grid map not publishing in offline mode
- Cause: RTAB-Map's `odom` topic remap does not create a subscription.
- Fix: Use `build_map.py` for offline processing instead of RTAB-Map.

### Robot doesn't respond to cmd_vel after ~3 minutes
- Cause: Gazebo sensor degradation over long sessions.
- Fix: Restart Gazebo, use headless mode (`gui:=false`), or record a bag and process offline.

### Right half of house misaligned in map
- Cause: VIO drift accumulated over 10+ minute trajectory.
- Fix: Use `build_map.py` with spatial correction parameters:
  `python3 build_map.py house.bak output -1 -1 -3.2 -1.25`
  Adjust corr_dx/corr_dy as needed based on visual inspection.

## Blog

Technical deep-dive (Chinese): [Pseudo-Stereo VIO + RTAB-Map 全链路 SLAM 系统搭建](https://soyorin.work/articles/000031.html)

## License

MIT
