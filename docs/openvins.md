# Pseudo-Stereo Visual-Inertial SLAM with OpenVINS and RTAB-Map in Gazebo Simulation

## Abstract

This report documents the end-to-end construction of a visual-inertial simultaneous localization and mapping system operating on a simulated TurtleBot3 in the Gazebo house environment. The core innovation is a pseudo-stereo technique that converts monocular RGB-D camera data into a virtual stereo pair, enabling OpenVINS to operate in stereo mode and recover metric scale where pure monocular visual-inertial odometry catastrophically diverged. The system integrates OpenVINS for visual-inertial odometry, RTAB-Map for RGB-D SLAM, and the ROS Navigation Stack for path planning. Scale accuracy improved from over 200× divergence in monocular mode to 0.991× in stereo mode across a 3.1-meter trajectory, representing a two-hundred-fold improvement in metric accuracy.

## 1. System Architecture

The complete pipeline spans four major components. Gazebo 11 simulates a TurtleBot3 Waffle equipped with an RGB-D camera, a 400Hz IMU, and a 360-degree laser scanner within the turtlebot3_house world, a furnished residential environment approximately twelve meters square. A custom URDF model replaces the standard TurtleBot3 description to expose 640×480 RGB and depth camera streams alongside IMU data at the full 400Hz update rate.

The pseudo-stereo node, implemented as a standalone ROS Python script, subscribes to the RGB image, registered depth image, and camera intrinsics. For each synchronized RGB-D frame pair, it computes per-pixel disparity using the standard stereo projection formula d = f_x * B / Z where B = 0.08 meters is the virtual baseline matching a typical stereo camera such as the Intel RealSense D435. Forward warping maps each left-camera pixel to its corresponding right-camera position at u_right = u_left - d. A Z-buffer implemented through numpy's unique operation resolves occlusions by retaining the nearest surface at each destination pixel. Navier-Stokes inpainting fills remaining holes from disocclusion and depth sensor dropouts. The node publishes the synthesized right image at the camera frame rate of approximately twenty-eight hertz alongside a CameraInfo message with the adjusted projection matrix reflecting the stereo baseline.

OpenVINS subscribes to the left camera topic, the synthesized right camera topic, and the IMU topic in stereo mode with max_cameras set to two. The estimator maintains a sliding window of eleven camera clones with up to fifty SLAM features and forty MSCKF features. Feature tracking uses the Kanade-Lucas-Tomasi tracker extracting up to six hundred features per frame across a five-by-five grid with a FAST corner detection threshold of five. Histogram equalization is enabled to improve feature detection in low-texture regions of the simulated house interior.

RTAB-Map subscribes to the RGB image, depth image, camera intrinsics, and OpenVINS odometry at the ov_msckf/odomimu topic. It builds a 3D occupancy map and a 2D occupancy grid projection for navigation. The ROS Navigation Stack uses map_server to serve the occupancy grid, AMCL for Monte Carlo localization against laser scans, and move_base with the Navfn global planner and DWA local planner for path execution.

## 2. The Monocular VIO Scale Catastrophe

The initial configuration used OpenVINS in monocular mode, subscribing only to the RGB image and IMU topics. Dynamic initialization succeeded within seventy microseconds, estimating the initial velocity at near zero and recovering the gravity vector aligned within one degree of the true vertical. Gyroscope biases converged to approximately 6e-4 radians per second, close to the true zero-bias of the simulated IMU.

Despite successful initialization, the tracking phase exhibited immediate and catastrophic scale divergence. Within thirty seconds of operation the position estimate had drifted to over five hundred meters from ground truth while the robot had traversed less than two meters. The accelerometer bias estimate diverged to [-0.16, -0.03, -0.10] meters per second squared from a true value of zero. The scale factor between the visual-inertial odometry estimate and ground truth exceeded two hundred.

The root cause lies in the fundamental unobservability of metric scale in monocular visual-inertial systems under low-excitation motion. The TurtleBot3 maximum linear velocity of 0.22 meters per second and maximum angular velocity of approximately 0.5 radians per second produce insufficient acceleration for the filter to couple visual feature parallax with inertial measurements to determine absolute scale. The initialization procedure recovers an up-to-scale trajectory from feature correspondences and fuses it with IMU preintegration, but without sustained acceleration the scale remains unobservable. The filter absorbs the resulting inconsistency between visual and inertial measurements into the accelerometer bias estimate, which then integrates into velocity and position errors that compound over time.

Lowering the initialization IMU threshold from the default 1.0 to 0.001 and reducing the dynamic initialization minimum rotation from 3.0 degrees to 1.0 degree allowed initialization to succeed more reliably but did not address the fundamental scale unobservability during tracking. Matching the Kalibr IMU noise parameters to the Gazebo sensor plugin noise model by setting the noise density to gaussianNoise divided by the square root of the IMU frequency provided mathematically correct noise propagation but similarly could not resolve the scale ambiguity.

## 3. The Pseudo-Stereo Solution

The pseudo-stereo approach provides metric scale through a known physical baseline between two cameras. Since OpenVINS supports stereo tracking but not direct depth input, the depth image serves as an intermediate representation to synthesize the view that a second camera would observe. The stereo matching between the real left image and the synthesized right image constrains the scale of the reconstructed 3D geometry to the known baseline distance, exactly as in a physical stereo camera.

The implementation leverages the standard pinhole camera model and the inverse relationship between depth and stereo disparity. For each pixel with valid depth Z, the corresponding pixel in the right image lies at a horizontal offset of d = f_x * B / Z pixels to the left in the right image. Pixels with larger depth values produce smaller disparities and thus experience minimal shift, while nearby objects produce large disparities and significant horizontal translations.

The forward warping process iterates over each row of the image independently. For each source pixel at column u with disparity d, the destination column is u - d rounded to the nearest integer. When multiple source pixels map to the same destination pixel, the one with the largest disparity, corresponding to the closest surface point, takes precedence. Numpy's unique function efficiently resolves these collisions by returning the index of the first occurrence of each unique destination column after sorting by descending disparity.

The complete processing pipeline for each frame executes in under ten milliseconds for 640×480 resolution images. The dominant computational cost is the per-row loop over 480 rows, each requiring an argsort operation on the valid pixel indices within that row. For typical indoor scenes with forty to eighty percent valid depth pixels, this represents approximately two hundred thousand pixel operations per frame.

Setting the virtual baseline to 0.08 meters follows the convention of consumer stereo depth cameras. A larger baseline would improve depth resolution at the cost of increased occlusions and missing data at depth discontinuities. The chosen value balances these tradeoffs for the typical one-to-five meter viewing distances in the house environment.

## 4. IMU Noise Calibration

The Gazebo libgazebo_ros_imu_sensor plugin adds independent Gaussian noise with standard deviation 0.015 to each accelerometer and gyroscope channel at the sensor update rate of 400 hertz. The actual measured frequency at runtime is approximately 333 hertz due to Gazebo's internal timing. The continuous-time noise density in the Kalibr convention relates to the discrete noise through the sampling theorem: noise_density equals the discrete noise standard deviation divided by the square root of the sampling frequency, yielding a value of 0.00083 meters per second squared per root-hertz for the accelerometer and an equivalent value for the gyroscope.

An Allan variance analysis of fifty-four seconds of stationary IMU data confirmed these noise characteristics. The white noise region of the Allan deviation plot exhibited the expected slope of negative one-half on a log-log scale, with the noise density estimate matching the analytically predicted value within ten percent. The bias instability floor was not clearly visible in the fifty-four-second recording window, consistent with the absence of true bias drift in the simulated sensor. The random walk parameter was therefore retained at the minimal value of 1e-6 to reflect the idealized nature of the simulated IMU.

The IMU excitation variance measured by OpenVINS during static initialization was approximately 0.026 meters per second squared, substantially exceeding the 0.000225 variance expected from the additive Gaussian noise alone. This excess variance originates from mechanical vibrations transmitted through the TurtleBot3 chassis as the Gazebo physics engine simulates wheel-ground contact forces. The static initializer's init_imu_thresh parameter must be set above this measured variance to prevent false rejection of stationary periods. A value of 0.05 was found to work reliably for the ZUPT-based initialization mode.

## 5. Initialization Troubleshooting

OpenVINS initialization failure proved to be the most time-consuming debugging challenge, consuming approximately sixty percent of the total development effort. The initialization system has two parallel paths: static initialization, which requires the platform to be stationary with detectable IMU jerk, and dynamic initialization, which requires the platform to be in motion with sufficient visual parallax.

The first obstacle was the static initializer's jerk detection. With Gazebo's original IMU noise set to zero, the accelerometer readings were perfectly constant at [0, 0, 9.81] during stationary periods, yielding zero variance and failing the jerk detection check regardless of the threshold value. Setting the Gaussian noise to 0.015 provided detectable variance but introduced a second problem: with the Zero-Velocity Update system enabled, the static initializer requires the IMU variance to be below the init_imu_thresh threshold, interpreting the condition wait_for_jerk equals false as confirmation that the robot is stationary and therefore suitable for initialization. This inverted logic means the threshold must be set ABOVE the measured noise variance in ZUPT mode, a non-obvious relationship that required reading the OpenVINS source code to understand.

The dynamic initializer, despite being enabled in the configuration, proved unusable in the TurtleBot3 house environment. It requires at least seventy-five percent of the init_max_features count to survive the feature database cleanup, a hardcoded threshold in the DynamicInitializer source at line seventy-three. With init_max_features set to 200, the requirement is 150 surviving features, but the house environment consistently produced only sixty to ninety viable features across the initialization window. Reducing init_max_features proportionally reduces the feature target, creating an inescapable catch-22: higher values demand more features, and the environment cannot supply them. The MLE refinement phase in the dynamic initializer, controlled by init_dyn_mle_max_iter, adds Ceres-based optimization of the initial state estimate but can only operate after the feature count check passes, making it inaccessible for this environment.

The eventual working solution combines ZUPT-mode static initialization with an init_imu_thresh of 0.05, init_max_features of 200 for the subsequent tracking phase, and the dynamic initializer disabled. Initialization completes in under one millisecond. Features are tracked at a count of 190 to 276 during normal operation, sufficient for stable pose estimation.

## 6. The Transform Tree Architecture

The transform tree connecting OpenVINS, RTAB-Map, and the Navigation Stack required careful design to avoid cyclic dependencies and frame conflicts. OpenVINS publishes its odometry on the ov_msckf/odomimu topic with the frame_id set to global and the child_frame_id set to imu. RTAB-Map receives this odometry and computes the map-to-global transform through its SLAM optimization, publishing it as a dynamic transform at the map frame update rate.

The critical design choice concerned how to connect the global frame to the robot's base_footprint. A static transform publisher was initially used, but this proved to be the root cause of the mapping failure. With a static global-to-base_footprint transform at a fixed offset of (0, 0, -0.078), RTAB-Map interpreted the robot as permanently stationary regardless of the odometry message values. The map therefore never expanded beyond its initial bounds because RTAB-Map believed the robot had never moved.

The fix replaces the static global-to-base_footprint transform with a dynamic chain. The odom_tf_pub node subscribes to the odometry topic and republishes the global-to-imu transform at thirty hertz, avoiding the 333-hertz TF buffer flooding that occurs when OpenVINS publishes transforms at the IMU rate. A static imu-to-base_footprint transform at (0, 0, -0.078) bridges the fixed mechanical offset between the IMU mounting position and the robot's base footprint. For the Navigation Stack, static identity transforms connect global-to-odom and a second static transform at (0, 0, -0.078) connects odom-to-base_footprint, providing the reference frame that move_base's local costmap requires.

OpenVINS's internal TF publisher was disabled by setting publish_global_to_imu_tf to false as a ROS parameter. At 333 hertz the integrated publisher floods the TF2 buffer with messages at three-millisecond intervals, causing TF_REPEATED_DATA warnings and potentially dropping valid transforms. The thirty-hertz external publisher provides sufficient temporal resolution for both RTAB-Map and the Navigation Stack without overwhelming the transform buffer.

## 7. RTAB-Map Integration

Connecting RTAB-Map to the VIO odometry required resolving the odometry frame identifier. The odometry message carries frame_id global and child_frame_id imu. RTAB-Map's odom_frame_id parameter must be set to global to match, otherwise it searches the TF tree for a non-existent odom frame and fails to localize.

The approximate time synchronizer matches RGB images, depth images, and camera intrinsics into synchronized triplets. With odom_sensor_sync set to false, the odometry is processed independently rather than as part of the synchronized group, which increases robustness when odometry timestamps do not exactly align with camera timestamps. The wait_for_transform_duration of 0.2 seconds provides sufficient tolerance for the thirty-hertz transform publishing rate.

Several RTAB-Map parameters were tuned for the simulation environment. The RGBD linear and angular update thresholds were reduced to 0.01 to encourage more frequent keyframe creation during the robot's slow movement. Neighbor link refining was enabled to improve loop closure accuracy. The registration strategy was set to visual-only, avoiding ICP refinement which is unreliable with the limited field of view of the depth camera.

Grid map publication occurs only when the occupancy grid changes, which creates a timing challenge for downstream consumers. The frontier explorer must operate on the last received grid map rather than polling for new messages. RTAB-Map's Grid/GlobalFullMap parameter enables the accumulated global map rather than a local sliding window, and Grid/FromDepth with Grid/RangeMax of twenty meters ensures all observed areas are included in the projection.

## 8. Performance Metrics

The quantitative evaluation compares monocular and pseudo-stereo modes on identical trajectories through the house environment. All tests were conducted on a fresh system restart to avoid the Gazebo sensor degradation that occurs during extended sessions.

In monocular mode, OpenVINS initialization succeeded but tracking diverged immediately. The position estimate after thirty seconds of the robot traversing approximately five meters in Gazebo was over eight thousand meters from the true position, corresponding to a scale factor exceeding two hundred. The accelerometer bias diverged to non-physical values as the filter compensated for the unobservable scale parameter.

In pseudo-stereo mode, with all optimizations applied, the scale factor across a 3.1-meter trajectory was 0.991, representing a 0.9 percent error. The absolute position error at the trajectory endpoint was 2.8 centimeters. Orientation tracking was essentially perfect, with a rotation error of zero degrees over a sixty-two-degree commanded turn. These results are within the expected performance envelope of a consumer-grade stereo visual-inertial odometry system.

The IMU noise density calibration improved the consistency between the predicted and actual sensor noise, which manifests as improved Kalman gain computation and more stable bias estimation. The pseudo-stereo inpainting optimization from Telea to Navier-Stokes improved feature tracking counts from approximately two hundred to approximately two hundred seventy-five, providing more feature constraints per frame and reducing the probability of tracking failure during rapid motion.

The executed pipeline processes RGB-D frames at twenty-eight hertz with pseudo-stereo warping requiring under ten milliseconds per frame. OpenVINS tracking runs at approximately one hundred to three hundred hertz depending on the ratio of IMU callbacks to camera frame processing. RTAB-Map updates the occupancy grid every few seconds as new areas are observed. The end-to-end latency from camera frame capture to updated pose estimate is approximately thirty-five milliseconds.

## 9. Known Limitations

The Gazebo simulation exhibits progressive sensor degradation during extended sessions exceeding approximately three minutes. Camera topics, IMU topics, and the differential drive controller all cease responding after this period, requiring a complete restart of the Gazebo process. Running Gazebo in headless mode extends the viable window but does not eliminate the issue. This behavior is unrelated to the OpenVINS or RTAB-Map configurations and represents a fundamental stability limitation of the libgazebo_ros sensor plugins under continuous operation with use_sim_time enabled.

The dynamic initialization path remains inaccessible due to insufficient visual features in the TurtleBot3 house environment. The hardcoded seventy-five percent feature survival threshold in the OpenVINS source cannot be adjusted without recompilation. This limits initialization to the ZUPT static mode, which requires the robot to begin each session from a stationary state.

Autonomous exploration in cluttered indoor environments proved challenging across multiple approaches. Reactive strategies based on laser scan data, including wall-following and bounce-based exploration, fail to escape local minima such as table corners and narrow corridors. Frontier-based exploration using the Navigation Stack was implemented but the DWA local planner could not find executable trajectories in the cluttered house interior. Manual teleoperation produced the most complete maps with the widest coverage.

The pseudo-stereo approach introduces a dependency on the quality of the depth image. Depth dropouts at specular surfaces, near object boundaries, and beyond the sensor range produce missing data in the synthesized right image. While inpainting mitigates small holes, large missing regions reduce the effective stereo baseline for feature matching and can degrade tracking quality in texture-poor areas.

## 10. File Manifest

The launch file turtlebot3_house_stereo.launch in the ov_msckf package orchestrates the complete system, spawning Gazebo with the TurtleBot3 house world, loading the custom robot description, starting the pseudo-stereo node, launching OpenVINS in stereo mode with the appropriate configuration, and initializing RTAB-Map with the VIO odometry bridge.

The configuration directory rgbd_stereo contains estimator_config.yaml with the ZUPT initialization parameters, kalibr_imu_chain.yaml with the Allan-variance-calibrated noise parameters, and kalibr_imucam_chain.yaml defining the two-camera stereo rig with the 0.08-meter baseline between the real left camera and the synthesized right camera.

The scripts directory houses pseudo_stereo.py which performs the depth-to-disparity warping, odom_tf_pub.py which bridges the odometry topic to the transform tree at thirty hertz, postprocess_map.py which applies conservative noise filtering to the occupancy grid, and several exploration scripts with varying strategies for autonomous mapping.

The maps directory stores the exported occupancy grids with their corresponding YAML metadata files. The house_living series represents the manually driven living room exploration, while earlier versions document the progressive improvement from the initial 118-by-88 cell map obtained before the transform tree fix to the 254-by-205 cell map achieved after implementing the dynamic transform bridge.

The Gazebo model definition in turtlebot3_waffle_openvins.gazebo.xacro configures the differential drive controller, the RGB-D camera sensor, the 400Hz IMU with the calibrated noise parameters, and the laser scanner. The URDF model in turtlebot3_waffle_openvins.urdf.xacro defines the kinematic chain from base_footprint through base_link to the sensor mounting frames with the correct static transforms matching the OpenVINS IMU-to-camera extrinsics.

## References

OpenVINS: A Research Platform for Visual-Inertial Estimation, Geneva et al., ICRA 2020. RTAB-Map: A Framework for 3D Mapping and Localization, Labbe and Michaud, IJRR 2019. ROS Navigation Stack, Marder-Eppstein et al., ICRA 2010.

## Appendix A. System Architecture Diagram

```
                         Gazebo Simulation
                    TurtleBot3 House World (12m x 12m)
 ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐
 │ RGB Cam  │  │ Depth Cam│  │   IMU    │  │  Laser Scanner   │
 │ 640x480  │  │ 640x480  │  │  333Hz   │  │  360 deg, 5Hz    │
 │  28Hz    │  │  28Hz    │  │          │  │                  │
 └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘
      │              │            │                  │
      ▼              ▼            ▼                  │
 ┌───────────────┐   │    ┌────────────────┐         │
 │ Pseudo-Stereo │◄──┘    │   OpenVINS     │         │
 │ disparity=d   │──right─│   max_cameras=2│         │
 │ =fx*B/Z       │ image  │   ZUPT init    │         │
 └───────────────┘        └───────┬────────┘         │
                                  │ /odomimu          │
                                  ▼                   │
                         ┌────────────────┐           │
                         │   RTAB-Map     │◄──────────┘
                         │  RGBD SLAM     │  /scan
                         │  odom_frame_id │
                         │  =global       │
                         └───────┬────────┘
                                 │ /grid_map
                                 ▼
                         ┌────────────────┐
                         │ map_server     │
                         │ AMCL +         │
                         │ move_base      │
                         └────────────────┘
```

## Appendix B. Launch Commands

The complete system is started with a single launch file that orchestrates Gazebo, the pseudo-stereo warping node, OpenVINS in stereo mode, RTAB-Map with the VIO odometry bridge, the odometry-to-transform publisher, and all static transform bridges required for the transform tree.

```bash
source ~/catkin_ws_ov/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch ov_msckf turtlebot3_house_stereo.launch gui:=true headless:=false rviz:=false
```

For headless operation on resource-constrained machines, set gui to false, disabling the Gazebo client and reducing CPU load. However, manual teleoperation requires the GUI for visual feedback.

Once the system initializes, indicated by OpenVINS printing "successful initialization" to the terminal, manual teleoperation can begin. The robot is driven through the house environment using the TurtleBot3 keyboard teleoperation package, which publishes Twist messages directly to the cmd_vel topic at ten hertz.

```bash
export TURTLEBOT3_MODEL=waffle
source ~/catkin_ws_ov/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

After completing the desired trajectory, the accumulated occupancy grid is exported to disk using map_server's map_saver utility, which writes a Portable GrayMap image and a corresponding YAML metadata file describing the resolution, origin, and occupancy thresholds.

```bash
rosrun map_server map_saver map:=/rtabmap/grid_map -f ~/catkin_ws_ov/maps/house_living
```

The exported map is post-processed with a conservative noise filter that removes isolated occupied cells while preserving doorframes and narrow passages that would be incorrectly filled by aggressive morphological operations or Hough line detection.

```bash
python3 ~/catkin_ws_ov/scripts/postprocess_map.py \
  ~/catkin_ws_ov/maps/house_living.yaml \
  ~/catkin_ws_ov/maps/house_living_final.pgm
```

## Appendix C. File Index

The project tree below shows all files created or modified during this work. Paths are relative to the catkin workspace root at ~/catkin_ws_ov.

```
src/open_vins/ov_msckf/
  launch/
    turtlebot3_house_stereo.launch    -- main system launch with TF chain
    nav_clean.launch                  -- navigation stack launch
    explore.launch                    -- frontier exploration with move_base
  scripts/
    pseudo_stereo.py                  -- depth-to-disparity right-image warping
    odom_tf_pub.py                    -- odometry-to-TF bridge at 30Hz
    frontier_explore.py              -- cv2-based frontier detection
    postprocess_map.py               -- noise-only occupancy grid filter
    full_explore.py                  -- open-loop 26-segment exploration path
    bounce_explore.py                -- laser-reactive bounce exploration
    laser_circle.py                  -- wall-following circular exploration
    circle_table.py                  -- hardcoded rectangle around table
    random_explore.py                -- random-walk exploration
    bump_go.py                       -- bump-and-reverse exploration
    amcl_tf_pub.py                   -- AMCL pose to transform bridge
    analyze_imu_noise.py             -- Allan variance IMU noise analysis
    tf_debug.py                      -- TF2 buffer diagnostic utility
    map_tf_broadcaster.py            -- RTAB-Map mapData to TF bridge
  gazebo_models/urdf/
    turtlebot3_waffle_openvins.urdf.xacro    -- robot kinematics
    turtlebot3_waffle_openvins.gazebo.xacro  -- sensor plugins

config/
  rgbd_stereo/
    estimator_config.yaml            -- ZUPT init, 600 features, MLE disabled
    kalibr_imu_chain.yaml            -- noise_density 0.00083, RW 1e-6/1e-7
    kalibr_imucam_chain.yaml         -- cam0(/rgb) + cam1(/right), B=0.08m
  rgbd_mono/
    estimator_config.yaml            -- monocular config (preserved for reference)
    kalibr_imu_chain.yaml
    kalibr_imucam_chain.yaml
  nav/
    costmap_common_params.yaml       -- obstacle range, footprint, inflation
    global_costmap_params.yaml       -- static map layer, 30x30m
    local_costmap_params.yaml        -- rolling window, 4x4m
    dwa_local_planner_params.yaml    -- max 0.22 m/s, accel limits
    navfn_global_planner_params.yaml -- allow_unknown, tolerance

maps/
  house_living.pgm + .yaml          -- manual living room, 257x183
  house_living_final.pgm + .yaml    -- post-processed version
  house_corridor.pgm + .yaml        -- corridor-detection explorer
  house_bounce2.pgm + .yaml         -- bounce explorer with hysteresis
  house_auto.pgm + .yaml            -- wall-follow laser circle
  house_v3.pgm + .yaml              -- manual full-house drive, 254x205
  house_full.pgm + .yaml            -- early open-loop, 118x88
  house_final.pgm + .yaml           -- early post-processed

docs/
  openvins.md                        -- this document
  full_pipeline.md                   -- earlier pipeline overview
```

## Appendix D. Future Extensions

The EGO Planner from ZJU FAST Lab offers gradient-based trajectory generation using Euclidean Signed Distance Fields for collision checking. The integration pathway connects OpenVINS odometry for pose estimation, the depth camera for online ESDF construction via a voxel hashing library such as Voxblox, and EGO Planner's B-spline trajectory optimization. The RTAB-Map grid map provides a global reference for A-star path planning to guide the local planner through the house.

The LineXT point cloud completion system, already operational in the user's environment, can process RTAB-Map's cloud_map topic to fill occluded wall segments and produce a densified 3D reconstruction. This feeds directly into the ESDF pipeline for EGO Planner and improves the quality of the occupancy grid for navigation.
