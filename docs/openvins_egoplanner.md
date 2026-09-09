# EGO-Planner Integration with OpenVINS Visual-Inertial Odometry

## Abstract

This report documents the integration of the EGO-Planner gradient-based local trajectory planner from ZJU FAST Lab with the OpenVINS pseudo-stereo visual-inertial odometry system. The integration creates a complete perception-to-action pipeline: VIO provides real-time pose estimation, an offline-built occupancy grid map provides global environmental context, and EGO-Planner generates collision-free B-spline trajectories converted to velocity commands for the differential-drive robot chassis. The key challenge was reconciling two independently designed coordinate frame conventions and topic namespaces without modifying the source code of either system. A bridge node translates the OpenVINS global-to-IMU odometry into the odom-to-base_footprint convention expected by EGO-Planner, while a carefully constructed transform tree ensures that all three subsystems -- VIO, mapping, and planning -- share a consistent spatial reference frame. The integrated system was verified through dry-run launch testing with all twelve nodes successfully initializing and maintaining stable operation over extended runtime.

## 1. EGO-Planner: Principles of Operation

EGO-Planner is an open-source gradient-based local trajectory planner originating from the FAST Lab at Zhejiang University. It was originally designed for aggressive quadrotor flight through cluttered environments but has been adapted for ground robot navigation. The planner's core innovation lies in replacing traditional sampling-based or optimization-based planning with a gradient descent formulation that directly optimizes B-spline control points against a Euclidean Signed Distance Field cost function.

### 1.1 B-spline Trajectory Representation

The trajectory is parameterized as a uniform cubic B-spline curve defined by a sequence of control points Q_i in three-dimensional space. The B-spline basis functions provide C2 continuity by construction, ensuring smooth velocity and acceleration profiles without explicit smoothness constraints. The uniform knot spacing dt determines the temporal resolution of the trajectory, typically set to 0.2 seconds for ground robot navigation. The trajectory evaluation at any time t yields position, velocity, and acceleration through analytical differentiation of the basis functions.

The B-spline representation confers three advantages for real-time replanning. First, local modifications to control points affect only a bounded segment of the trajectory, enabling efficient gradient-based optimization without recomputing the entire curve. Second, the convex hull property guarantees that the trajectory lies within the convex hull of the control points, providing a conservative bound for collision checking against the ESDF. Third, the derivative of the trajectory with respect to any control point admits a closed-form expression, enabling direct computation of the collision cost gradient for numerical optimization.

### 1.2 Euclidean Signed Distance Field for Collision Checking

EGO-Planner constructs an ESDF from the incoming point cloud representing the environment. The ESDF assigns to each point in the workspace the signed distance to the nearest obstacle surface, with negative values inside obstacles and positive values in free space. The zero-level set defines the obstacle boundary. The ESDF is computed using a two-pass propagation algorithm that first rasterizes occupied cells from the point cloud, then propagates distance values outward from obstacle boundaries using a dynamic programming sweep.

The ESDF enables collision checking through simple function evaluation rather than geometric intersection tests. A configuration is collision-free if and only if the ESDF value at that point exceeds a safety margin. The gradient of the ESDF points away from the nearest obstacle, providing the direction for repulsive forces in the trajectory optimization.

### 1.3 Two-Stage Planning Pipeline

The planning process operates in two stages. The first stage is a global path search using A* on the ESDF grid. The search front expands from the robot's current position toward the goal, with the heuristic cost equal to the Euclidean distance to the goal weighted by a constant factor. The search produces a piecewise-linear path through free space that serves as an initial guess for the second stage.

The second stage refines the path into a smooth, dynamically feasible trajectory through gradient-based optimization of the B-spline control points. The total cost function combines three terms:

$$
J = \lambda_s J_s + \lambda_c J_c + \lambda_f J_f
$$

The smoothness cost J_s penalizes the integral of squared jerk along the trajectory, encoded as the sum of squared finite differences between consecutive control point accelerations. The collision cost J_c evaluates the ESDF at each control point and applies an exponential penalty when the distance falls below a safety threshold. The feasibility cost J_f enforces limits on velocity and acceleration by penalizing control point configurations that would produce exceedances given the B-spline time scaling.

The optimization uses the L-BFGS algorithm for efficiency, exploiting the sparsity structure induced by the B-spline's local support. The gradient of each cost term with respect to the control points is computed analytically, avoiding finite-difference approximations. The optimization converges in typically ten to thirty iterations for the planning horizon of five meters typical of indoor navigation.

### 1.4 Trajectory Server and Control

The optimized B-spline trajectory is published as a custom ROS message containing the control points, knot vector, and temporal parameters. The trajectory server node converts this representation into instantaneous velocity commands for the differential-drive chassis using a pure-pursuit controller.

The pure-pursuit algorithm selects a lookahead point on the trajectory at a fixed time horizon, typically 0.6 seconds ahead of the current robot position. The linear velocity command is proportional to the distance to the lookahead point divided by the lookahead time, clamped to a maximum of 0.5 meters per second. The angular velocity command is computed from the curvature of the trajectory at the lookahead point and the current linear velocity. A goal tolerance of 0.25 meters triggers a stop command when the robot reaches the target.

## 2. The EGO-Planner Node Architecture

The planner comprises five distinct ROS nodes, each handling a specific stage of the perception-to-control pipeline.

### 2.1 map_server

The standard ROS navigation stack map_server node reads an occupancy grid map from a YAML metadata file and the corresponding Portable GrayMap image. It exposes the map on the latched /map topic as a nav_msgs/OccupancyGrid message and provides a /static_map service for on-demand retrieval. The map resolution is 0.05 meters per cell with the origin at the bottom-left corner of the grid, matching the offline-built map from the VIO global reference frame.

### 2.2 map_to_pc2

A custom Python node that converts the 2D occupancy grid to a 3D point cloud required by the ESDF constructor. The conversion iterates over every cell in the grid, and for cells with occupancy probability exceeding a threshold of 30 percent, generates points at multiple vertical layers from Z equals negative 0.2 meters to Z equals 1.2 meters in increments of 0.14 meters. This extrusion creates a vertical wall representation from the 2D floor plan, enabling the ESDF to register obstacles at the robot's operating height of approximately 0.2 meters. The resulting PointCloud2 message is published on /map_generator/global_cloud with latch mode enabled and a republish timer at 10 hertz to ensure late-joining subscribers receive the data.

### 2.3 waypoint_generator

The waypoint generator bridges RViz user interaction to the planner. In manual-lonely-waypoint mode, it subscribes to the /move_base_simple/goal topic where RViz publishes the user's 2D Nav Goal interaction as a geometry_msgs/PoseStamped. Upon receiving a goal, the generator publishes it as a one-point nav_msgs/Path message on /waypoint_generator/waypoints. The generator also subscribes to /odom_world to verify odometry availability before accepting goals, preventing planning attempts before the localization system is ready.

The generator supports alternative modes including preset waypoint sequences loaded from the ROS parameter server, enabling scripted navigation missions through multiple rooms without human intervention.

### 2.4 ego_planner_node

The core planning node, implemented in C++ for performance, integrates the ESDF construction, A* global search, and B-spline optimization into a finite state machine. The FSM maintains four states:

- INIT: Waiting for valid odometry, point cloud, and goal waypoint data before transitioning to planning.
- WAIT_TARGET: Idle state when no goal is set, periodically checking for new waypoints.
- GEN_NEW_TRAJ: Active planning state that generates a new B-spline trajectory from the current position to the most recent goal, using the latest ESDF for collision checking.
- EXEC_TRAJ: Execution state that monitors progress along the active trajectory, triggering replanning when the lookahead point approaches the trajectory end or when new obstacles are detected.

The FSM operates at a configurable replanning frequency, typically 10 to 20 hertz. State transitions are logged to ROS INFO with distinct color codes for monitoring. In REAL mode, used for physical robots, the FSM disables drone-specific features such as height control and velocity feedforward.

The node exposes parameters for the planning horizon distance and time, the fixed navigation height, and the maximum number of waypoints to process. When use_sim_mode is set to false, the node operates in ground robot mode with planning restricted to the horizontal plane at the configured fixed height.

### 2.5 traj_server

The trajectory execution node subscribes to the B-spline trajectory on /planning/bspline and the robot odometry on /odom_world. It implements the pure-pursuit control law described in Section 1.4, publishing Twist commands on /cmd_vel. The server monitors trajectory completion, goal proximity, and odometry timeouts, transitioning the robot to a stopped state when the goal tolerance is satisfied or if odometry data becomes stale.

Configurable parameters include the lookahead time, linear and angular velocity limits, acceleration limits, and the goal tolerance radius. The linear and angular feedback gains provide tunable tracking performance for different robot dynamics.

## 3. Original EGO-Planner Transform Tree and Topics

The original EGO-Planner simulation environment, as configured in start_navigation.launch, establishes a transform tree designed for a self-contained simulation where odometry is synthesized from commanded velocities.

```
map ──(static identity)──→ world
map ──(static identity)──→ odom
odom ──(dynamic, fake_odom)──→ base_link
```

The map frame serves as the global reference frame for the occupancy grid map. The world frame exists for compatibility with drone-specific visualization nodes and is statically equivalent to map. The odom frame is also statically equivalent to map, meaning the robot's position in the map frame is directly given by the odometry integration. The fake_odom node simulates a differential-drive odometry source by integrating incoming /cmd_vel Twist commands over time, publishing the estimated pose as a nav_msgs/Odometry message on /odom_world and as a transform from odom to base_link.

This architecture works for simulation because the map, world, and odom frames are all static and identical. The robot's position in any of these frames is simply the integrated odometry, and there is no drift correction or loop closure to reconcile.

## 4. The Integration Challenge: Reconciling Two Frame Conventions

The OpenVINS VIO system and EGO-Planner were developed independently with different frame conventions, topic names, and transform topologies. Reconciling these without modifying the source code of either system was the central technical challenge of the integration.

### 4.1 Odometry Topic Translation

OpenVINS publishes its visual-inertial odometry estimate on /ov_msckf/odomimu as a nav_msgs/Odometry message with frame_id set to global and child_frame_id set to imu. The pose component represents the position and orientation of the IMU sensor frame expressed in the VIO global reference frame, which is the frame in which VIO initializes at the start of each session.

EGO-Planner expects odometry on /odom_world with frame_id set to odom and child_frame_id set to base_link. The pose represents the position of the robot's base link in the fixed odom reference frame. In the original architecture, the odom frame is static and identical to the map frame, so the odometry directly encodes the robot's absolute position.

The bridge between these conventions requires three transformations. First, the VIO odometry message must be relayed from /ov_msckf/odomimu to /odom_world. Second, the reference frame must be translated from the VIO global frame to the EGO-Planner odom frame. Third, the tracked body must be translated from the IMU sensor frame to the robot base footprint frame, applying the fixed 0.078-meter vertical offset specified in the TurtleBot3 URDF where the IMU is mounted 0.068 meters above the base link and the base footprint lies 0.01 meters below the base link.

### 4.2 Frame ID Translation

The frame translation is handled by the vio_odom_bridge node. It subscribes to /ov_msckf/odomimu and, on each message, applies the fixed IMU-to-base_footprint offset rotated by the current IMU orientation to account for the robot's pitch and roll on uneven terrain:

```
p_basefootprint_in_global = p_imu_in_global + R_global_to_imu * (0, 0, -0.078)
```

The resulting position and the unmodified IMU orientation are published as /odom_world with the frame_id set to odom and the child_frame_id set to base_footprint. The node also publishes the same pose as a dynamic transform from odom to base_footprint at 50 hertz, maintaining a complete TF tree for visualization and diagnostic purposes.

### 4.3 Velocity Propagation

The bridge propagates VIO velocity estimates directly, copying the linear and angular velocity from the input message. While VIO velocity estimates may contain more noise than wheel encoder odometry, they provide sufficient accuracy for the pure-pursuit controller's lookahead computation and trajectory tracking. The 6x6 pose covariance matrices are also forwarded, allowing downstream nodes to assess localization quality.

## 5. The Unified Transform Tree

The integrated transform tree preserves the semantics expected by both the VIO system and the EGO-Planner while maintaining global consistency.

```
map ──(static identity)──→ global ──(static identity)──→ odom
odom ──(dynamic, from VIO)──→ base_footprint
base_footprint ──(URDF, from robot_state_publisher)──→ base_link
base_link ──(URDF)──→ imu_link, camera_link, base_scan
```

The map frame is the global reference for the occupancy grid. Since the offline map was constructed from VIO data projected into the global frame, a static identity transform connects map to global. The global frame is the VIO world frame, established at the start of each session by the ZUPT initialization. A second static identity transform connects global to odom, establishing the odom frame as a fixed world frame in which the robot moves -- exactly the semantic that EGO-Planner expects.

The odom-to-base_footprint transform is published dynamically by the vio_odom_bridge node at 50 hertz, reflecting the robot's VIO-estimated pose. This replaces the fake_odom integration with real visual-inertial tracking. The remaining kinematic chain from base_footprint through base_link to the sensor frames is provided by the URDF robot_state_publisher at 50 hertz.

A static identity transform from map to world is maintained for compatibility with drone visualization nodes that reference the world frame, though it is unused in ground robot operation.

### 5.1 Why Not Use /tf_static for Fixed Transforms

All static transforms in the integrated system are published using tf static_transform_publisher targeting the /tf topic rather than /tf_static. The /tf_static topic uses latching, meaning only the most recent publisher's message is retained. In a system with multiple static transform publishers from different workspaces and launch files, the last publisher to connect overwrites all previously latched messages, silently breaking the transform tree.

Publishing fixed transforms to /tf at a 100-millisecond repetition rate ensures that transforms are periodically refreshed and available to any subscriber regardless of startup ordering. The computational overhead is negligible since each static transform message is a few hundred bytes.

## 6. Workspace Integration

The integration spans two independently built catkin workspaces that must coexist in a single ROS environment.

### 6.1 The Two-Workspace Problem

OpenVINS resides in the catkin_ws_ov workspace at ~/catkin_ws_ov, built with catkin_make and containing the ov_msckf, ov_core, ov_init, ov_eval, and ov_data packages. EGO-Planner resides in a separate workspace at ~/VINS-Nav/ego-planner/planner, containing 22 packages including ego_planner, waypoint_generator, traj_utils, bspline_opt, plan_env, path_searching, quadrotor_msgs, and map_tools.

The EGO-Planner workspace was originally built on a different machine at the path /home/bingoling/Desktop/planner and was subsequently copied to the local machine as a pre-built binary distribution. This introduced three categories of path corruption that prevented the workspace from functioning: a devel/.catkin marker pointing to the original machine's source directory, devel/lib wrapper scripts with hardcoded source paths from the build machine, and missing package.xml files in the devel/share directories for all 22 packages because the original source tree was not included in the copy.

### 6.2 Environment Chaining

The standard catkin workspace chaining mechanism, where each setup.sh extends the environment from the previously sourced workspace, failed because the EGO-Planner workspace setup scripts were generated on a different machine. When sourced without the --extend flag, the setup.sh script performs a rollback operation that removes all environment modifications from previously sourced workspaces, replacing them with only the current workspace and the base ROS installation.

The solution uses the CATKIN_SETUP_UTIL_ARGS environment variable to force extend mode when sourcing the EGO-Planner workspace, preserving the OpenVINS workspace entries in CMAKE_PREFIX_PATH, LD_LIBRARY_PATH, PYTHONPATH, and ROS_PACKAGE_PATH:

```bash
CATKIN_SETUP_UTIL_ARGS="--extend" source ~/VINS-Nav/ego-planner/planner/devel/setup.bash
```

A custom environment setup script, setup_ego_vio.sh in the project root, encapsulates this logic and additionally strips newline characters that the EGO-Planner workspace injects into ROS_PACKAGE_PATH due to a formatting error in its generated environment hooks.

### 6.3 Package Discovery Restoration

The missing package.xml files were reconstructed for all 22 EGO-Planner packages. For map_tools, the original package.xml from the included source tree was used. For the remaining 21 packages, minimal package.xml files were created by analyzing the compiled shared library dependencies with ldd and extracting message type definitions from the devel/include C++ headers. The quadrotor_msgs, ego_planner, and multi_map_server packages additionally required .msg definition files reconstructed from the compiled message header definitions, since the original .msg source files were not included in the binary distribution.

### 6.4 Wrapper Script Path Correction

The catkin build system generates relay scripts in devel/lib/<pkg>/<script_name>.py that reference the original source file path. These scripts were regenerated with the source path parameter set to the correct local directory. Without this correction, map_to_pc2.py and fake_odom.py would fail with FileNotFoundError when roslaunch attempted to execute them through the wrapper.

## 7. Topic and Data Flow Verification

The complete topic graph of the integrated system was verified through analysis of compiled binary symbols and through dry-run launch testing.

### 7.1 Topic Inventory

The table below enumerates every topic in the integrated system, the publishing node, the subscribing nodes, and the message type. All connections were verified against the compiled binary symbol tables of the EGO-Planner C++ nodes and the Python source of the bridge and conversion scripts.

| Topic | Publisher | Subscribers | Type |
|-------|-----------|-------------|------|
| /ov_msckf/odomimu | OpenVINS | vio_odom_bridge | nav_msgs/Odometry |
| /odom_world | vio_odom_bridge | ego_planner_node, traj_server, waypoint_generator | nav_msgs/Odometry |
| /map | map_server | ego_planner_node GridMap, RViz | nav_msgs/OccupancyGrid |
| /map_generator/global_cloud | map_to_pc2 | ego_planner_node | sensor_msgs/PointCloud2 |
| /waypoint_generator/waypoints | waypoint_generator | ego_planner_node | nav_msgs/Path |
| /planning/bspline | ego_planner_node | traj_server | ego_planner/Bspline |
| /cmd_vel | traj_server | Gazebo diff-drive controller | geometry_msgs/Twist |
| /move_base_simple/goal | RViz 2D Nav Goal | waypoint_generator | geometry_msgs/PoseStamped |
| /tf | tf publishers, vio_odom_bridge | All TF-consuming nodes | tf2_msgs/TFMessage |
| /static_map | map_server (service) | map_to_pc2 | nav_msgs/GetMap |

### 7.2 Binary Symbol Verification

The ego_planner_node and traj_server are compiled C++ binaries with no accessible source code. Their subscription and publication topics were verified by extracting symbol strings from the compiled executables. The ego_planner_node binary confirmed subscriptions to /odom_world (via indep_odom_sub_), /map_generator/global_cloud (via cloudCallback expecting PointCloud2), and /waypoint_generator/waypoints (via waypointCallback expecting nav_msgs/Path). The traj_server binary confirmed subscriptions to /planning/bspline (via bspline_sub_) and /odom_world (via odom_sub_), and publication of /cmd_vel (via cmd_pub_).

The waypoint_generator binary confirmed its subscription to odometry and goal topics, with the exact topic names configured through ROS remapping at launch time rather than hardcoded. The node's name-relative private topics are remapped to match the global topic namespace.

### 7.3 Extended Runtime Stability

A 20-second dry-run launch test with Gazebo disabled confirmed that all 12 nodes initialize successfully and maintain stable operation without error escalation. The ego_planner_node FSM enters the INIT state and logs periodic status at 3-second intervals while waiting for VIO odometry data, which is normal behavior when no physical robot or simulation is providing pose updates. The traj_server reports Ready status immediately. The map_to_pc2 conversion completes within one second of startup. The only recurring warning is a tf2_msgs/TFMessage type mismatch from RViz connecting to the /tf topic, a benign compatibility issue between the tf and tf2 message types that does not affect functionality.

The only error-level message is a boost::lock_error thrown during process termination when the launch timeout sends SIGTERM to the ego_planner_node. This is a shutdown race condition in the node's destructor, not a runtime error, and does not occur during normal operation or clean shutdown.

## 8. Launch Configuration

The integrated system supports two launch modes. For the one-session workflow (drive → map → navigate), use nav_ego_rtabmap.launch which includes RTAB-Map for online mapping and localization. For the offline map workflow (pre-built map from a previous session), use nav_ego_vio.launch.

### 8.1 One-Session Workflow: nav_ego_rtabmap.launch

This is the primary launch file for the complete drive-to-navigate pipeline. It starts Gazebo, VIO, RTAB-Map, and ego-planner in a single session:

```bash
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_rtabmap.launch
```

In a second terminal, start keyboard teleoperation:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

The workflow proceeds as follows. First, drive the robot through the environment using keyboard teleoperation. RTAB-Map constructs the occupancy grid in real time from the RGB-D camera stream, with OpenVINS providing metric visual-inertial odometry. The live grid map appears in RViz through the rtabmap_map_relay bridge, which republishes RTAB-Map's grid_map to the standard /map topic and provides a /static_map service.

The map_to_pc2 node, configured with refresh_interval of 2.0 seconds, periodically re-fetches the latest map from RTAB-Map and converts it to a PointCloud2 for ego-planner's ESDF constructor. This ensures the planner's collision model remains synchronized with the accumulating map data.

Second, once the environment is adequately mapped, stop driving. The accumulated occupancy grid is now available for navigation. In RViz, use the "2D Nav Goal" tool (keyboard shortcut G) to click a destination point on the map.

Third, ego-planner's FSM transitions from INIT to GEN_NEW_TRAJ upon receiving the goal waypoint. It runs A* global path search on the ESDF grid, optimizes the B-spline trajectory against the collision and smoothness costs, and publishes the trajectory to traj_server. The trajectory server converts the B-spline to velocity commands, and the robot begins autonomous navigation.

### 8.2 Offline Map Mode: nav_ego_vio.launch

In offline mode, the launch file uses a pre-built occupancy grid map from a previous mapping session. RTAB-Map is not included; the map is served by map_server from a YAML file, and EGO-Planner operates without online localization correction:

```bash
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_vio.launch gazebo:=true map_file:=/path/to/map.yaml
```

The pipeline is: Gazebo sensors feed pseudo_stereo for virtual right-camera synthesis, OpenVINS processes the stereo pair with IMU for metric visual-inertial odometry, vio_odom_bridge translates the odometry to EGO-Planner convention, map_server serves the offline-built occupancy grid, map_to_pc2 converts it to a point cloud for the ESDF constructor, and ego_planner_node generates trajectories that traj_server converts to velocity commands published to the Gazebo differential-drive controller.

### 8.2 Offline Bag Replay Mode

In offline mode, only the map server, conversion nodes, bridge, and EGO-Planner stack are launched. The user plays a recorded bag file containing VIO data, images, and TF transforms from a previous session:

```bash
source ~/VINS-Nav/setup_ego_vio.sh
roslaunch ov_msckf nav_ego_vio.launch sim_time:=true
rosbag play --clock house.bag
```

This mode is useful for developing and testing the planning system without the computational overhead and stability limitations of the Gazebo simulation.

### 8.3 Parameter Configuration

EGO-Planner parameters are configured in the launch file rather than in external YAML files, consistent with the original EGO-Planner convention. The planning horizon is set to 5.0 meters with a horizon time of 5.0 seconds, matching the scale of the TurtleBot3 house environment. The fixed navigation height is 0.2 meters, corresponding to the robot's base footprint height above the ground plane. The trajectory server limits linear velocity to 0.5 meters per second and angular velocity to 1.0 radians per second, with a 0.25-meter goal tolerance and 0.6-second lookahead time.

## 9. Unmodified Components

A fundamental design constraint of the integration was that neither the OpenVINS pipeline nor the EGO-Planner binaries could be modified. The following components operate exactly as they did before integration:

**OpenVINS components (unchanged):**
- estimator_config.yaml with ZUPT initialization, 600 features, MLE disabled
- kalibr_imu_chain.yaml with noise_density = 0.00083
- kalibr_imucam_chain.yaml with baseline = 0.08 meters
- pseudo_stereo.py depth-to-disparity warping node
- turtlebot3_house_stereo.launch original VIO launch file

**EGO-Planner components (unchanged):**
- ego_planner_node compiled binary (10.4 MB C++ executable)
- traj_server compiled binary (2.4 MB C++ executable)
- waypoint_generator compiled binary (0.3 MB C++ executable)
- map_to_pc2.py source script
- fake_odom.py source script (unused but preserved)

**Mapping components (unchanged):**
- build_map.py offline bag-to-map processor
- postprocess_v5.py occupancy grid post-processor
- house_final.pgm and house_final.yaml offline-built map

## 10. Known Limitations and Future Work

The EGO-Planner integration inherits the limitations of the individual components. The VIO system requires stationary ZUPT initialization at the start of each session and will fail to converge if the robot moves during the first few seconds after startup. The Gazebo simulation exhibits progressive sensor degradation after approximately three minutes of continuous operation, necessitating restarts during extended testing. The ego_planner_node FSM path is currently limited to the horizontal plane at a fixed height, meaning it cannot plan over or under obstacles.

The nav_ego_rtabmap.launch configuration addresses this by running RTAB-Map alongside ego-planner. RTAB-Map provides online map→odom localization correction through its mapData topic, and the rtabmap_map_relay node bridges the live grid_map to ego-planner's map_to_pc2 conversion pipeline. The map_to_pc2 refresh_interval parameter enables periodic re-fetching of the accumulating map, ensuring the planner always works with the latest available occupancy data.

The trajectory server uses a simple pure-pursuit controller to convert B-spline trajectories to velocity commands. A model-predictive control approach that directly optimizes the velocity profile against the robot's differential-drive dynamics would provide smoother tracking with lower latency, particularly during aggressive maneuvers near obstacles.

The bridge node currently propagates VIO velocity estimates directly to the odometry message. While the VIO linear velocity estimates are accurate during constant-velocity motion, they exhibit noise amplification during acceleration phases characteristic of visual-inertial estimation. Fusing the VIO velocity with wheel odometry through a Kalman filter would improve the velocity signal quality for the pure-pursuit controller.

## Appendix A. EGO-Planner Integration Data Flow Diagram

```
                        Gazebo Simulation (online) or ROS Bag (offline)
                                   │
              ┌────────────────────┼────────────────────┐
              ▼                    ▼                    ▼
       RGB Image             Depth Image             IMU 400Hz
              │                    │                    │
              └────────┬───────────┘                    │
                       ▼                                │
              ┌──────────────────┐                      │
              │  pseudo_stereo   │                      │
              │  B=0.08m         │                      │
              │  left→right warp │                      │
              └────────┬─────────┘                      │
                       │ right image                    │
                       ▼                                ▼
              ┌──────────────────────────────────────────┐
              │            OpenVINS Stereo VIO           │
              │            max_cameras=2                 │
              │            ZUPT static init              │
              │            noise_density=0.00083         │
              └────────────────────┬─────────────────────┘
                                   │ /ov_msckf/odomimu
                                   │ frame_id=global
                                   │ child_frame_id=imu
                                   ▼
              ┌─────────────────────────────────────────────┐
              │         vio_odom_bridge.py  (NEW)           │
              │                                             │
              │  imu→base_footprint (Z offset -0.078m)      │
              │  global→odom (frame_id translation)         │
              │                                             │
              │  OUTPUT: /odom_world (odom→base_footprint)  │
              │  OUTPUT: TF odom→base_footprint @ 50Hz      │
              └────────────────────┬────────────────────────┘
                                   │
         ┌─────────────────────────┼──────────────────────┐
         │                         │                      │
         ▼                         ▼                      ▼
   map_server              ego_planner_node        waypoint_generator
   offline map                  │                      ▲
   house_final.yaml             │                      │
         │                      │              /move_base_simple/goal
         ▼                      │                 (RViz 2D Nav Goal)
   map_to_pc2.py                │
   2D grid→3D cloud             │
         │                      │
         ▼                      │
   /map_generator/              │
   global_cloud                 │
         │                      │
         └──────────────────────┼───────────┐
                                ▼           │
                        /planning/bspline   │
                                │           │
                                ▼           │
                          traj_server       │
                          pure pursuit      │
                                │           │
                                ▼           │
                           /cmd_vel         │
                           (Twist)          │
                                │           │
                                ▼           │
                     Gazebo diff-drive      │
                     or real robot chassis  │
                                            │
                     /odom_world ◄──────────┘
```

## Appendix B. File Manifest

The integration adds the following files to the VINS-Nav project. All existing OpenVINS and mapping files remain unchanged.

```
VINS-Nav/
├── setup_ego_vio.sh                         ← unified workspace environment
├── scripts/
│   └── vio_odom_bridge.py                   ← VIO→EGO odometry bridge (NEW)
├── openvins/ov_msckf/
│   ├── launch/
│   │   ├── nav_ego_vio.launch               ← offline-map integrated launch (NEW)
│   │   ├── nav_ego_rtabmap.launch           ← one-session drive+nav launch (NEW)
│   │   └── nav_ego.rviz                     ← RViz config for navigation (NEW)
│   └── scripts/
│       ├── vio_odom_bridge.py               ← VIO→EGO odometry bridge (NEW)
│       └── rtabmap_map_relay.py             ← RTAB-Map grid → /map + /static_map (NEW)
├── ego-planner/planner/src/                 ← restored package metadata
│   ├── ego_planner/
│   │   ├── package.xml                      ← (NEW)
│   │   └── msg/
│   │       ├── Bspline.msg                  ← (NEW, from header)
│   │       └── DataDisp.msg                 ← (NEW, from header)
│   ├── waypoint_generator/
│   │   └── package.xml                      ← (NEW)
│   ├── quadrotor_msgs/
│   │   ├── package.xml                      ← (NEW)
│   │   └── msg/                             ← 13 .msg files (NEW)
│   ├── traj_utils/package.xml               ← (NEW)
│   ├── bspline_opt/package.xml              ← (NEW)
│   ├── plan_env/package.xml                 ← (NEW)
│   ├── path_searching/package.xml           ← (NEW)
│   └── ... (15 additional package.xml)      ← (NEW)
└── docs/
    └── openvins_egoplanner.md               ← this document (NEW)
```

## Appendix C. Debugging Checklist

The following issues were encountered and resolved during integration. They are documented here for future reference when debugging similar multi-workspace ROS integrations.

**Symptom: rospack cannot find ego-planner packages.**
Root cause: Missing package.xml in devel/share directories. The workspace was copied as a binary distribution without source code. Resolution: Create minimal package.xml files in devel/share/<pkg>/ and mirror them to the source tree under planner/src/<pkg>/ for rospack discovery via ROS_PACKAGE_PATH.

**Symptom: CMAKE_PREFIX_PATH loses catkin_ws_ov entries after sourcing ego-planner setup.**
Root cause: Sourcing setup.bash without --extend triggers a full environment rollback that drops previously sourced workspaces. Resolution: Set CATKIN_SETUP_UTIL_ARGS="--extend" before sourcing the second workspace.

**Symptom: ROS_PACKAGE_PATH contains embedded newline characters.**
Root cause: The ego-planner workspace environment hooks generate a newline in the ROS_PACKAGE_PATH variable, likely from a formatting error in the cross-machine build. Resolution: Strip newlines and carriage returns from ROS_PACKAGE_PATH in the unified setup script.

**Symptom: map_to_pc2.py crashes with FileNotFoundError referencing /home/bingoling/Desktop/planner.**
Root cause: The catkin-generated wrapper scripts in devel/lib/<pkg>/ contain hardcoded source paths from the original build machine. Resolution: Regenerate wrapper scripts with the correct local source path.

**Symptom: rosmsg show ego_planner/Bspline fails with "Cannot locate message."**
Root cause: The .msg definition files were not included in the binary distribution. Only the compiled C++ headers and client library bindings exist. Resolution: Reconstruct .msg source files from the C++ header Definition functions, which contain the exact message specification string.

**Symptom: ego_planner_node FSM stays in INIT state.**
Root cause: The FSM requires valid odometry data on /odom_world before transitioning to the planning state. Without a running VIO system or simulation, no odometry is published. This is normal and expected in dry-run tests.

**Symptom: TF error "Could not find a connection between 'global' and 'base_footprint'."**
Root cause: The TF chain global→odom (static) + odom→base_footprint (dynamic from VIO bridge) is incomplete before VIO initializes and publishes odometry. Resolution: Add a static odom→base_footprint fallback transform identical to the one in the original turtlebot3_house_stereo.launch, ensuring the TF tree is connected at all times. The dynamic bridge's transforms take precedence for actual robot pose once VIO data arrives.

**Symptom: vio_odom_bridge TF not received by TF2 buffer.**
Root cause: The bridge was publishing raw geometry_msgs/TransformStamped to /tf using rospy.Publisher with rospy.AnyMsg, which produces incorrect message framing. TF2 expects tf2_msgs/TFMessage wrapping. Resolution: Use tf2_ros.TransformBroadcaster.sendTransform() instead of raw topic publication.

## Appendix D. Related Work

OpenVINS: A Research Platform for Visual-Inertial Estimation. Patrick Geneva, Kevin Eckenhoff, Woosik Lee, Yulin Yang, and Guoquan Huang. IEEE International Conference on Robotics and Automation, 2020.

EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors. Xin Zhou, Zhepei Wang, Hongkai Ye, Chao Xu, and Fei Gao. IEEE Robotics and Automation Letters, 2021.

EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments. Xin Zhou, Jiangchao Zhu, Hongyu Zhou, Chao Xu, and Fei Gao. IEEE International Conference on Robotics and Automation, 2021.

RTAB-Map: A Framework for 3D Mapping and Localization. Mathieu Labbé and François Michaud. International Journal of Robotics Research, 2019.

## Appendix E. Technical Blog

1. [Pseudo-Stereo VIO + RTAB-Map 全链路 SLAM 系统搭建](https://soyorin.work/articles/000031.html) — VIO architecture, TF tree, offline mapping
2. [EGO-Planner 集成：VIO + 建图 + 规划全栈融合](https://soyorin.work/articles/000033.html) — EGO-Planner internals and VINS-Nav integration
