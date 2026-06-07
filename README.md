# VINS-Nav: Pseudo-Stereo Visual-Inertial SLAM + Navigation

> [中文](README_CN.md)

Gazebo TurtleBot3 house environment with OpenVINS stereo VIO, RTAB-Map RGBD SLAM,
and ROS Navigation Stack. The core innovation is a depth-to-stereo warping node that
enables metric-scale stereo VIO from a single RGB-D camera.

**Scale accuracy: 0.991x** (vs. 200× divergence in monocular mode).  
**3.1m trajectory drift: 2.8cm (0.9%).**  
**Orientation error: 0.0°.**

## Architecture

```
Gazebo TurtleBot3 House (12m x 12m)
├── RGB-D Camera (640x480, 28Hz)
├── IMU (333Hz, gaussianNoise=0.015)
└── Laser Scanner (360°, 5Hz)

Pipeline:
  RGB + Depth → pseudo_stereo.py → right camera (disparity warp)
  Left + Right + IMU → OpenVINS (stereo, max_cameras=2) → /ov_msckf/odomimu
  RGB + Depth + Odometry → RTAB-Map → /rtabmap/grid_map
  Grid Map → map_server + AMCL + move_base → Navigation

TF Chain:
  map → global → odom → base_footprint → base_link → sensors
```

## Requirements

- Ubuntu 20.04, ROS Noetic
- TurtleBot3 packages: `ros-noetic-turtlebot3-gazebo`, `ros-noetic-turtlebot3-teleop`
- RTAB-Map: `ros-noetic-rtabmap-ros`
- Navigation: `ros-noetic-move-base`, `ros-noetic-amcl`, `ros-noetic-map-server`
- OpenVINS: compiled from source in a catkin workspace
- Python: numpy, opencv-python, pyyaml, rosbag

## File Inventory

### Modified from upstream OpenVINS

| File | Change |
|------|--------|
| `openvins/config/rgbd_stereo/` | New stereo config directory (estimator, IMU, camera chain) |
| `openvins/ov_msckf/launch/turtlebot3_house_stereo.launch` | Main launch: Gazebo + pseudo-stereo + OV + RTAB-Map |
| `openvins/ov_msckf/launch/rtabmap_db.launch` | RTAB-Map node with odom remap |
| `openvins/ov_msckf/launch/nav_clean.launch` | Navigation stack (AMCL + move_base) |
| `openvins/ov_msckf/launch/explore.launch` | Frontier exploration with move_base |
| `openvins/ov_msckf/launch/offline_bag.launch` | Offline bag processing pipeline |
| `openvins/ov_msckf/launch/turtlebot3_house_openvins.launch` | Original monocular config (preserved) |

### New OpenVINS package scripts

| Script | Purpose |
|--------|---------|
| `ov_msckf/scripts/pseudo_stereo.py` | Depth-to-disparity right-image warping (baseline=0.08m) |
| `ov_msckf/scripts/dyn_odom_tf.py` | Dynamic global→odom TF from VIO odometry (30Hz) |
| `ov_msckf/scripts/odom_tf_pub.py` | Odomimu→TF bridge (30Hz, for RTAB-Map) |
| `ov_msckf/scripts/frontier_explore.py` | cv2-based frontier detection (no sklearn) |
| `ov_msckf/scripts/amcl_tf_pub.py` | AMCL pose→TF publisher |
| `ov_msckf/scripts/map_tf_broadcaster.py` | RTAB-Map mapData→TF bridge |

### New Gazebo models

| File | Purpose |
|------|---------|
| `ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.urdf.xacro` | Robot kinematics with RGB-D + IMU |
| `ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.gazebo.xacro` | Sensor plugins (gaussianNoise=0.015, publishOdomTF=false) |

### Navigation config

| File | Purpose |
|------|---------|
| `openvins/config/nav/costmap_common_params.yaml` | Common costmap (obstacle_range, footprint) |
| `openvins/config/nav/global_costmap_params.yaml` | Global costmap (static, 30x30m) |
| `openvins/config/nav/local_costmap_params.yaml` | Local costmap (rolling, 4x4m) |
| `openvins/config/nav/dwa_local_planner_params.yaml` | DWA planner (max 0.22m/s) |
| `openvins/config/nav/navfn_global_planner_params.yaml` | Navfn global planner |

### Standalone tools (scripts/)

| Script | Purpose |
|--------|---------|
| `build_map.py` | Offline map builder: bag → OV poses + depth → occupancy grid |
| `postprocess_v5.py` | Map post-processing: denoise, boundary fill, narrow gap protection, drift mask |
| `postprocess_map.py` | Lightweight map post-processing |
| `analyze_imu_noise.py` | Allan variance IMU noise analysis |
| `offline_final.sh` | One-shot offline pipeline (pseudo_stereo + OV + RTAB-Map + bag play) |
| `bounce_explore.py` | Laser-reactive bounce exploration |
| `laser_circle.py` | Wall-following circular exploration |
| `tf_debug.py` | TF2 buffer diagnostic |

## Usage

### Online (Gazebo live)

```bash
source ~/catkin_ws_ov/devel/setup.bash
export TURTLEBOT3_MODEL=waffle

# Launch full system
roslaunch ov_msckf turtlebot3_house_stereo.launch

# Manual teleop
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Export map
rosrun map_server map_saver map:=/rtabmap/grid_map -f house_map
```

### Offline (bag processing)

```bash
# Record bag (sensors-only, no odometry needed)
rosbag record -O house.bag \
  /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info \
  /imu /clock /tf /tf_static

# Build map directly from bag
python3 scripts/build_map.py house.bak output_map

# Post-process
python3 scripts/postprocess_v5.py output_map.yaml output_map_final.pgm
```

### Navigation

```bash
roslaunch ov_msckf nav_clean.launch
```

## Key Configuration

`openvins/config/rgbd_stereo/estimator_config.yaml`:
- `try_zupt: true`, `init_imu_thresh: 0.05`
- `init_max_features: 200`, `num_pts: 600`
- `init_dyn_mle_max_iter: 0` (MLE disabled — insufficient features in simulation)

`openvins/config/rgbd_stereo/kalibr_imu_chain.yaml`:
- `noise_density: 0.00083` (Allan variance measured)
- `random_walk: 1e-6 / 1e-7`

`openvins/config/rgbd_stereo/kalibr_imucam_chain.yaml`:
- `cam0: /camera/rgb/image_raw`, `cam1: /camera/right/image_raw`
- `baseline: 0.08m`

## Known Issues

1. **VIO drift over long trajectories**: Right-half of the house drifts ~3m after 10+ minutes. Corrected in `build_map.py` via spatial split (`ox < -1.0` correction). For online use, loop closure would require RTAB-Map's graph optimization (works in live mode, broken in offline mode).

2. **Gazebo sensor degradation**: Camera and IMU topics stop publishing after ~3 minutes in GUI mode. Workaround: headless mode (`gui:=false`) or bag recording + offline processing.

3. **Blocked doorways**: Narrow passages (<0.2m) may appear as walls in the occupancy grid. The `build_map.py` wall detection (`gw*3 > gf`) is conservative; lowering the ratio reduces false walls but also thins real walls.

4. **RTAB-Map offline mode**: The `odom` topic remap does not create a subscription in offline bag playback. RTAB-Map's graph optimization produces zero poses. For offline mapping, use `build_map.py` instead.

## Performance

| Metric | Value |
|--------|-------|
| Scale ratio | 0.991x |
| 3.1m drift | 2.8cm (0.9%) |
| Orientation error | 0.0° |
| Init time | <1ms |
| Features tracked | 190–276 |
| Map coverage | 398×317 cells (19.9m×15.9m) |
| Occupied cells | 14,512 |

## Team

- VIO & Mapping: [your name]
- Navigation: [team member]
- EGO Planner: [team member]

## License

MIT
