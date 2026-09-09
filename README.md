# VINS-Nav: Pseudo-Stereo Visual-Inertial SLAM + Navigation

> [中文](README_CN.md)

End-to-end visual-inertial SLAM and navigation on a simulated TurtleBot3 in Gazebo.
The core innovation is **pseudo-stereo**: a depth-to-disparity warping node that generates
a virtual right camera from a single RGB-D sensor, enabling OpenVINS to operate in
metric-scale stereo mode.

Navigation is handled by **EGO-Planner** (ZJU FAST Lab), integrated without modifying
either codebase — a bridge node translates OpenVINS odometry into the frame convention
EGO-Planner expects. The full loop is: drive to map → click a goal in RViz → autonomous
B-spline trajectory → `/cmd_vel`.

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
6. [EGO-Planner Integration](#ego-planner-integration)
7. [Script Reference](#script-reference)
8. [Configuration Reference](#configuration-reference)
9. [File Inventory](#file-inventory)
10. [Performance](#performance)
11. [Troubleshooting](#troubleshooting)

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

Optional autonomous navigation (EGO-Planner), see §EGO-Planner Integration:
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
                          │  2D→3D cloud   │           │
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
                                     robot chassis
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

Navigation Layer (AMCL + move_base, alternative):
  map_server:  IN: (file)  OUT: /map (static, latched)
  AMCL:        IN: /map, /scan  OUT: /amcl_pose, /tf (map→odom)
  move_base:   IN: /map, /scan, /tf  OUT: /cmd_vel

EGO-Planner Layer (optional, see §EGO-Planner Integration):
  vio_odom_bridge.py:
    IN:  /ov_msckf/odomimu
    OUT: /odom_world, TF odom→base_footprint

  rtabmap_map_relay.py:
    IN:  /rtabmap/grid_map
    OUT: /map, service /static_map

  map_to_pc2 (from ego-planner):
    IN:  service /static_map
    OUT: /map_generator/global_cloud

  waypoint_generator:
    IN:  /move_base_simple/goal (RViz "2D Nav Goal"), /odom_world
    OUT: /waypoint_generator/waypoints

  ego_planner_node:
    IN:  /odom_world, /map_generator/global_cloud, /waypoint_generator/waypoints
    OUT: /planning/bspline

  traj_server:
    IN:  /planning/bspline, /odom_world
    OUT: /cmd_vel
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
| `/odom_world` | nav_msgs/Odometry | 50Hz | EGO-Planner odometry bridge (odom→base_footprint) |
| `/map` | nav_msgs/OccupancyGrid | on change | Live map relayed from `/rtabmap/grid_map` (latched) |
| `/map_generator/global_cloud` | sensor_msgs/PointCloud2 | 10Hz | 2D grid extruded to 3D cloud for EGO-Planner's ESDF |
| `/waypoint_generator/waypoints` | nav_msgs/Path | on goal | Navigation goal from RViz "2D Nav Goal" |
| `/planning/bspline` | ego_planner/Bspline | on replan | Optimized B-spline trajectory |
| `/cmd_vel` (EGO mode) | geometry_msgs/Twist | 20Hz | Pure-pursuit velocity command from traj_server |

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

### TF Tree in EGO-Planner Mode

EGO-Planner expects `odom` to be a **fixed world frame** with the robot moving inside it
(same semantic as the original `fake_odom`). The chain becomes:

```
map ──(RTAB-Map mapData, dynamic)──► odom ──(VIO bridge, dynamic)──► base_footprint ──(URDF)──► base_link
```

| Transform | Publisher | Type | Notes |
|-----------|-----------|------|-------|
| `map → odom` | `map_tf_broadcaster.py` | dynamic, 10Hz | RTAB-Map localization correction from `/rtabmap/mapData` |
| `odom → base_footprint` | `vio_odom_bridge.py` | dynamic, 50Hz | VIO pose with imu→base Z offset applied |
| `global → odom` | static publisher | static identity | required because RTAB-Map's `odom_frame_id=global` |
| `imu → base_footprint` | static publisher | static | fallback, Z = −0.078 m |

**Why a static `odom → base_footprint` fallback is also present:** before VIO initializes
there is no dynamic transform, and TF2 reports *"Could not find a connection between
'global' and 'base_footprint'"*. A static fallback keeps the tree connected; the dynamic
bridge takes over as soon as odometry arrives.

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

### EGO-Planner Workspace (only for §EGO-Planner Integration)

EGO-Planner lives in a **second** catkin workspace, checked out as a git submodule:

```bash
git submodule update --init --recursive      # from the repo root
cd ego-planner/planner
catkin_make                                  # or use the prebuilt devel/ if present
```

Two workspaces must be overlaid, and the second one must be sourced in *extend* mode
or it will roll back the first one's environment:

```bash
source /home/nu/VINS-Nav/setup_ego_vio.sh    # handles both + the --extend flag
```

`setup_ego_vio.sh` also strips newlines that the EGO-Planner workspace injects into
`ROS_PACKAGE_PATH`, which otherwise makes `rospack` fail to find every package.

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

### 3. Navigation (AMCL + move_base)

```bash
# Launch navigation stack with a pre-built map
roslaunch ov_msckf nav_clean.launch

# Send a goal
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}" \
  "pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}"
```

### 4. Drive → Map → Navigate (EGO-Planner, one session)

```bash
# Terminal 1: everything — Gazebo, VIO, RTAB-Map, EGO-Planner
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_rtabmap.launch

# Terminal 2: drive the robot around to build the map
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Terminal 3 (optional): watch the map grow
rostopic hz /map

# When the map looks good: stop driving.
# In RViz press G, click a destination → EGO-Planner plans and drives there.
```

The map is live: RTAB-Map's grid is relayed to `/map` and `/static_map`, and
`map_to_pc2` re-reads it every 2 s so the planner's ESDF stays current.

For a **pre-built map** instead (no RTAB-Map, no online localization):

```bash
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_vio.launch gazebo:=true map_file:=/path/to/map.yaml
```

### 5. Replay a recorded bag (no Gazebo)

The same stack runs against a recorded bag — VIO re-runs live on the replayed
camera/IMU stream, so no simulator is needed:

```bash
# Terminal 1: VIO + RTAB-Map + EGO-Planner, no Gazebo
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_rtabmap.launch gazebo:=false rviz:=false

# Terminal 2: replay ONLY sensor topics + clock
rosbag play ~/house_full.bag --clock \
  --topics /camera/rgb/image_raw /camera/depth/image_raw \
           /camera/rgb/camera_info /imu /clock
```

**Do not replay `/tf` or `/tf_static`.** The bag's TF was produced by the original
session's odometry and would fight the live `odom → base_footprint` published by
`vio_odom_bridge.py`, producing conflicting transforms on the same edge.

Two gotchas when testing this way:

- Publish navigation goals with a **persistent** publisher. A one-shot
  `rostopic pub -1` can be dropped before the subscriber connection is established;
  `waypoint_generator` then silently never fires. RViz's "2D Nav Goal" is fine.
- Expect **VIO drift**. `pseudo_stereo` is the throughput bottleneck (see
  §Performance), so the estimator receives far fewer stereo frames than it needs.

---

## EGO-Planner Integration

[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) (ZJU FAST Lab) is a
gradient-based local planner: A* search over a Euclidean Signed Distance Field (ESDF)
produces a coarse path, which is then refined into a smooth, collision-free **B-spline**
trajectory by optimizing its control points against smoothness and collision costs.
`traj_server` converts the B-spline into `/cmd_vel` via pure-pursuit control.

### Design constraint: neither side was modified

The integration adds a bridge layer only. OpenVINS config, `pseudo_stereo.py`, the
EGO-Planner binaries, `map_to_pc2.py` and `fake_odom.py` are untouched. The one
exception is a **6-line backward-compatible addition** to `map_to_pc2.py` (a
`refresh_interval` parameter, default `0` = original behavior) so it can re-read a
live-updating map.

### The mismatch being bridged

| | OpenVINS | EGO-Planner |
|---|----------|-------------|
| Odometry topic | `/ov_msckf/odomimu` | `/odom_world` |
| `frame_id` | `global` | `odom` |
| `child_frame_id` | `imu` | `base_footprint` |
| Map source | RTAB-Map `/rtabmap/grid_map` | `/map` + `/static_map` service |

`vio_odom_bridge.py` subscribes to `/ov_msckf/odomimu`, applies the fixed
IMU→base_footprint offset (0.078 m down, rotated by the current IMU orientation),
and republishes as `/odom_world` **plus** a dynamic `odom → base_footprint` TF at 50 Hz.

`rtabmap_map_relay.py` republishes RTAB-Map's `/rtabmap/grid_map` to `/map` and
advertises a `/static_map` service, because RTAB-Map does not provide one and
`map_to_pc2` depends on it.

### Launch files

| Launch file | Map source | RTAB-Map | Use case |
|-------------|-----------|----------|----------|
| `turtlebot3_house_stereo.launch` | RTAB-Map | ✔ | Mapping only (original) |
| `nav_ego_rtabmap.launch` | RTAB-Map, live | ✔ | **Drive → map → navigate, one session** |
| `nav_ego_vio.launch` | `map_server` (file) | ✘ | Pre-built map navigation |

### Verified topic contract

Every topic connection below was confirmed against the compiled EGO-Planner binary
symbol tables (there is no source in the distributed build) and by launch testing:

```
waypoint_generator  sub: /move_base_simple/goal, /odom_world   pub: /waypoint_generator/waypoints
ego_planner_node    sub: /odom_world, /map_generator/global_cloud, /waypoint_generator/waypoints
                    pub: /planning/bspline  (ego_planner/Bspline)
traj_server         sub: /planning/bspline, /odom_world         pub: /cmd_vel
map_to_pc2          sub: service /static_map                    pub: /map_generator/global_cloud
```

### Workspace restoration notes

The EGO-Planner workspace is distributed **pre-built**, without source. On a fresh
clone, run the restoration script once:

```bash
./scripts/setup_ego_workspace.sh        # idempotent, safe to re-run
```

It regenerates everything from artifacts that *are* present (compiled C++ headers and
shared libraries):

1. `devel/.catkin` repointed to the local source dir.
2. `package.xml` written for all 19 packages in `devel/share/` (and `src/`, without
   clobbering the real `map_tools/package.xml`) — otherwise `rospack` finds nothing.
3. `*.msg` definitions reconstructed from the C++ `Definition` structs, so `rosmsg
   show` and any rebuild work. (The distributed build ships only compiled headers.)
4. catkin wrapper scripts under `devel/lib/*/` repointed from the original build
   machine's path (`/home/bingoling/Desktop/planner/...`) to the local source.

The full debugging history is in `docs/openvins_egoplanner.md`.

### Known limitation

The `ego_planner_node` FSM stays in `INIT` until odometry arrives on `/odom_world`.
Without a running VIO (or a bag replay) it will simply idle — that is expected, not a
failure. VIO also needs a few stationary seconds for ZUPT initialization before it
publishes anything.

---

## Script Reference

### `pseudo_stereo.py` — Virtual Stereo Generator

Generates a virtual right camera image from RGB + depth using the stereo disparity formula.

**Input topics:** `/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`
**Output topics:** `/camera/right/image_raw`, `/camera/right/camera_info`
**Algorithm:** For each depth pixel with value Z, compute disparity d = fx * baseline / Z. The right-image pixel at column (u − d) takes the left-image value from column u. A per-row Z-buffer resolves occlusions, implemented with `np.lexsort((-disp, u_dst))` plus a boolean group-start diff (pixel-identical to the earlier `np.unique` approach but ~25% faster). Remaining disocclusion holes are filled according to `~fill_mode`.
**Baseline:** 0.08m (matching Intel RealSense D435).

**Hole filling (`~fill_mode`)** — the right image is ~40% holes at typical indoor
depth, and the original full-resolution Navier-Stokes inpainting cost ~170 ms/frame,
capping the node at ~5 Hz and starving VIO. Measured per 640×480 frame:

| `fill_mode` | Cost | Rate | Notes |
|-------------|------|------|-------|
| `morph` (default) | 40.2 ms | 24.9 Hz | morphological close, dilates real pixels into holes |
| `none` | 32.8 ms | 30.5 Hz | holes stay black; safest for stereo matching |
| `inpaint_ds` | 49.3 ms | 20.3 Hz | TELEA at 1/4 scale, upsampled into the holes |
| `inpaint_ns` | 201.5 ms | 5.0 Hz | original behavior, kept for comparison |

With the default, `/camera/right/image_raw` went from 4.4 Hz to ~16 Hz (input-limited).
Measured on a 900 s bag replay:

| Configuration | Result |
|---------------|--------|
| VIO only (pseudo-stereo + OpenVINS) | **0 NaN over the full 900 s**; previously NaN at ~450 s |
| Full stack (+ RTAB-Map + EGO-Planner) | NaN appears again after ~500 s (pseudo-stereo drops to ~13 Hz under the extra load) |

So the throughput fix removes the bottleneck when VIO runs alone, but the full stack
still needs headroom before mapping stays clean — see §Troubleshooting.

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

**Pose-jump correction — off by default, measured harmful.**
`build_map.py` can detect single-sample discontinuities (`|dp|/dt` above `v_max`,
default 1.0 m/s) and offset every later pose. Validated against wheel odometry, this
makes the trajectory **worse**, not better: the backward snaps are the estimator
correcting a stretch that had run ahead, so re-applying the offset re-injects the
error. Enable with `BUILD_MAP_JUMP_FIX=1` only for other datasets.

### How accurate is the VIO, really?

Validated against wheel odometry integrated from the bag's wheel joints
(`base_link → wheel_left/right_link`; radius 0.033 m, separation 0.287 m):

| Window | VIO vs wheel |
|--------|--------------|
| t = 252–873 s | segment scale **0.977 / 1.046 / 1.022 / 1.020 / 1.009** |
| t = 850–955 s | local alignment scale **0.9950**, RMSE **0.020 m** |
| t ≈ 955–975 s | error grows 0.07 m → **3.55 m**; ground truth shows the robot moving slowly (0.11 m/s, constant heading) while the VIO reports ~2× the motion |
| t ≈ 1121–1245 s | scale 1.12 |

So the VIO is near-exact for most of the run, and the map separation comes from **two
localized episodes**, not from a global scale error. Over the whole bag the VIO covers
90.7 m of path where the wheels turned 79.4 m.

> Caveat: wheel odometry is itself an integration and drifts slowly; it is a good
> relative reference over these windows, not an absolute ground truth.

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

### `vio_odom_bridge.py` — VIO → EGO-Planner Odometry Bridge

Translates OpenVINS odometry into the frame convention EGO-Planner expects.

**Input:** `/ov_msckf/odomimu` (`global → imu`)
**Output:** `/odom_world` (`odom → base_footprint`) at 50 Hz, plus dynamic TF `odom → base_footprint`

**Transform:** `p_base = p_imu + R(imu) · (0, 0, −0.078)` — the fixed offset from the
IMU (0.078 m above `base_footprint`) rotated into the world frame. Orientation is
passed through unchanged (IMU and base_footprint share it). Pose/twist covariances
are forwarded so downstream nodes can gauge localization quality.

Uses `tf2_ros.TransformBroadcaster` — publishing a bare `TransformStamped` to `/tf`
does **not** produce a valid `tf2_msgs/TFMessage` and TF2 silently ignores it.

### `rtabmap_map_relay.py` — Live Map Relay

Bridges RTAB-Map's live occupancy grid to the interfaces EGO-Planner needs.

**Input:** `/rtabmap/grid_map`
**Output:** `/map` (latched) + service `/static_map`

RTAB-Map publishes `grid_map` but neither `/map` nor a `/static_map` service, and
`map_to_pc2` calls `/static_map` on startup. Without this relay the planner gets an
empty world.

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

### RTAB-Map grid parameters

The stock Noetic install links OctoMap, so `Grid/3D` defaults to **true** — the 2D grid
is then projected from an octree and **2D ray tracing is not applied**. With ray tracing
off the grid only counts hits: one bad depth point stays occupied forever, which is the
mechanism behind the radial streak artefacts.

| Parameter | Stock | Here | Why |
|-----------|-------|------|-----|
| `Grid/3D` | true | **false** | take the 2D path so ray tracing applies; also cheaper |
| `Grid/RayTracing` | false | **true** | carves free space, erases transient false points |
| `Grid/RangeMax` | 5.0 | **4.5** | measured sensor clip in the bag; 20 invited far outliers |
| `Grid/RangeMin` | 0 | **0.2** | drop near flying pixels |
| `Grid/DepthRoiRatios` | 0 0 0 0 | **0 0 0 0.26** | rows 355–479 of every depth frame are 100% invalid |
| `Grid/NoiseFilteringRadius` | 0.0 | **0.1** | outlier removal (was disabled) |
| `Grid/NoiseFilteringMinNeighbors` | 5 | 5 | keep |
| `Grid/MaxGroundHeight` | 0.0 | **0.20** | robot-height ground segmentation |

**Measured facts about the depth stream** (worth knowing before tuning):
the valid fraction is a constant **62.5% = exactly 300/480 rows** — the bottom 26% of
every frame is 100% invalid, and max range is clipped at 4.49 m. That is structural,
not sensor noise, and no morphological close can fill it.

**A/B on a 400 s replay of `house_full.bag`** (identical input, only the grid params
differ):

| | map size | occupied | free |
|---|---|---|---|
| stock params | 264×227 | 15,035 | 6,318 |
| params above | 236×215 | 4,577 | 22,855 |

The stock map is dense with radial streaks and speckle; the new one has clean walls and
properly carved free space. The circular boundary is simply the 4.5 m ray range.

> **But structure matters more than cleanliness.** Compared against a hand-annotated
> floor plan (`maps/ground_truth_annotated.png`, rendered side by side in
> `docs/map_quality_comparison.png`):
>
> | version | house structure |
> |---------|-----------------|
> | hand-annotated truth | complete: outer walls, partitions, furniture, doorway |
> | **offline `build_map.py`** | **closest to truth** — rooms and furniture clearly readable |
> | RTAB-Map stock params | an unreadable black mass |
> | RTAB-Map new params | streaks gone, **but walls are fragmented and the structure is incomplete** |
>
> Ray tracing removed the streaks but also erodes thin walls and low furniture. For a
> usable map, the current recommendation is **offline `build_map.py` for the map,
> RTAB-Map only for live localisation (`map→odom`)**. Live mapping still needs finer
> tuning or a different front end to match the offline result.

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
    │   ├── nav_ego_rtabmap.launch            # EGO-Planner + live RTAB-Map map
    │   ├── nav_ego_vio.launch                # EGO-Planner + pre-built map
    │   ├── nav_ego.rviz                      # RViz config (Fixed Frame: map)
    │   └── explore.launch                    # Frontier exploration
    └── scripts/                      # New scripts
        ├── pseudo_stereo.py          # Depth→right camera warping
        ├── dyn_odom_tf.py            # Dynamic global→odom TF
        ├── odom_tf_pub.py            # Odom→TF bridge
        ├── frontier_explore.py       # cv2 frontier detection
        ├── amcl_tf_pub.py            # AMCL→TF bridge
        ├── map_tf_broadcaster.py     # RTAB-Map→TF bridge
        ├── vio_odom_bridge.py        # VIO→EGO-Planner odometry bridge
        └── rtabmap_map_relay.py      # RTAB-Map grid→/map + /static_map
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
├── vio_odom_bridge.py    # VIO→EGO-Planner odometry + TF bridge
├── rtabmap_map_relay.py  # RTAB-Map grid→/map + /static_map
└── tf_debug.py           # TF2 buffer diagnostic tool

setup_ego_vio.sh          # Combined catkin_ws_ov + ego-planner environment
ego-planner/              # git submodule (ZJU-FAST-Lab EGO-Planner)
docs/openvins_egoplanner.md  # Full integration report (architecture + debugging log)
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

**Pseudo-stereo throughput is the system bottleneck** (measured 2026-09-09, isolated
VIO-only replay of `house_full.bag`, so CPU contention is excluded):

| Stage | Cost per 640×480 frame |
|-------|------------------------|
| Row-wise forward warp | 62.9 ms |
| `cv2.inpaint(INPAINT_NS)` | **223.0 ms** |
| **Total** | **285.9 ms → 3.5 Hz ceiling** |

The recorded depth is only 54.8% valid, so the inpainting mask is large and
Navier-Stokes inpainting dominates. VIO wants ~28 Hz stereo pairs and gets ~3.5 Hz,
which degrades tracking, accumulates drift, and eventually produces NaN poses. This is
the root cause of the "map splits and needs manual `corr_dx`/`corr_dy`" symptom.

EGO-Planner stack (launch-tested; full closed-loop navigation depends on VIO quality):

| Metric | Value | Notes |
|--------|-------|-------|
| Nodes started | 19/19 | Gazebo + VIO + RTAB-Map + planner |
| Odometry bridge rate | 50 Hz | `/odom_world` |
| Map refresh period | 2 s | `map_to_pc2` re-reads live RTAB-Map grid |
| TF errors after fallback | 0 | was 100+ before the static `odom→base_footprint` fallback |
| Planning horizon | 5.0 m / 5.0 s | EGO-Planner FSM |
| Max linear / angular | 0.5 m/s / 1.0 rad/s | `traj_server` limits |
| Goal tolerance | 0.25 m | pure-pursuit stop radius |

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

### `rospack` cannot find any EGO-Planner package
- Cause: the pre-built workspace ships without `package.xml` in `devel/share/*/`.
- Fix: see §EGO-Planner Integration → *Workspace restoration notes*, and use
  `setup_ego_vio.sh` rather than sourcing the workspaces by hand.

### `ROS_PACKAGE_PATH` looks correct but `rospack` still fails
- Cause: the EGO-Planner workspace injects embedded newline characters into
  `ROS_PACKAGE_PATH`, which corrupts path parsing.
- Fix: `export ROS_PACKAGE_PATH=$(echo "$ROS_PACKAGE_PATH" | tr -d '\n\r')`
  (already handled inside `setup_ego_vio.sh`).

### Sourcing the second workspace wipes the first one's environment
- Cause: catkin's `setup.bash` rolls back previously sourced workspaces unless told
  to extend.
- Fix: `CATKIN_SETUP_UTIL_ARGS="--extend" source <second-ws>/devel/setup.bash`.

### TF error: *"Could not find a connection between 'global' and 'base_footprint'"*
- Cause: before VIO initializes, the dynamic `odom → base_footprint` transform does not
  exist yet, so the tree is split in two.
- Fix: keep a static `odom → base_footprint` fallback in the launch file; the dynamic
  bridge overrides it once odometry arrives.

### Bridge publishes TF but TF2 never sees it
- Cause: publishing a raw `geometry_msgs/TransformStamped` to `/tf` — the topic carries
  `tf2_msgs/TFMessage`, so the message is dropped.
- Fix: use `tf2_ros.TransformBroadcaster().sendTransform()`.

### `ego_planner_node` stays in `INIT` forever
- Cause: it waits for odometry on `/odom_world`; with no VIO running there is none.
  Also expected in a dry-run test.
- Fix: start the VIO (or replay a bag) and confirm `rostopic hz /odom_world` is non-zero.
  VIO needs a few stationary seconds for ZUPT initialization first.

### A bridge/relay node starts and logs "ready", but its topic has no data
- Cause: the node crashed in `__init__` *after* registering with the master. A classic
  instance: `NameError: name 'tf2_ros' is not defined` — the process exits in
  milliseconds, so `rosnode list` briefly shows it and the launch log looks fine.
- Fix: always verify with `rostopic hz <topic>`, not with "the node started".
  Check the node's own log file under `~/.ros/log/<run>/` for a traceback, and note
  that `rospy.Timer.run()` does **not** wrap callbacks in try/except — an exception in
  a timer callback kills the timer thread silently.

### `/map_tf_broadcaster` dies with `ROSTimeMovedBackwardsException`
- Cause: `rospy.Rate.sleep()` raises when `/clock` jumps (bag replay start/stop).
- Fix: catch `ROSTimeMovedBackwardsException` and rebuild the `rospy.Rate` against the
  new clock (already handled in `map_tf_broadcaster.py`).

### `waypoint_generator` receives a goal but never publishes a waypoint
- Cause: the goal was sent with a one-shot publisher (`rostopic pub -1`); the message
  can be dropped before the subscriber connection is established, and the callback
  never fires.
- Fix: keep the publisher alive (RViz "2D Nav Goal" does this naturally), or publish
  repeatedly for a few seconds. Verify the connection exists with
  `rosnode info /waypoint_generator` while the publisher is running.

### TF warnings `TF_DENORMALIZED_QUATERNION ... (nan nan nan nan)`
- Cause: VIO lost tracking and is emitting NaN poses — usually because
  `pseudo_stereo` is starving it of stereo frames (see §Performance).
- Fix: check `/camera/right/image_raw` rate with `rostopic hz`; if it is a few Hz
  instead of ~28 Hz, fix the pseudo-stereo bottleneck first.

## Blog

Technical deep-dive (Chinese):

1. [Pseudo-Stereo VIO + RTAB-Map 全链路 SLAM 系统搭建](https://soyorin.work/articles/000031.html)
2. [EGO-Planner 集成：VIO + 建图 + 规划全栈融合](https://soyorin.work/articles/000033.html)

## License

MIT
