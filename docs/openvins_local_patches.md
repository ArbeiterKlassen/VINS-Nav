# Local patches to OpenVINS (`~/catkin_ws_ov/src/open_vins/`)

The upstream OpenVINS source is not vendored in this repository, but two local
edits are required for the system to work on this machine. Both were found by
debugging why Gazebo produced no odometry at all.

## 1. ZUPT IMU-buffer trim window: 0.10 s → 0.50 s

**File:** `ov_msckf/src/core/VioManager.cpp`, four occurrences of
`clean_old_imu_measurements(... - 0.10)` in the ZUPT branches.

**Why:** the branch trims the IMU buffer to the last 0.10 s. The propagation that
publishes odometry starts from the cached state time, which only advances on a
camera update. At the 8–14 Hz the Gazebo camera actually delivers on this VM
(70–125 ms per frame) the cached time falls outside the trimmed buffer, so
`select_imu_readings` finds nothing and logs
`No IMU measurements to propagate with (0 of 2). IMU-CAMERA are likely messed up!!!`

**Effect of the patch:** that warning went from continuous to two occurrences per
run.

## 2. Gazebo depth sensor `update_rate`

**File:** `ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.gazebo.xacro`

The `<sensor type="depth">` block had no `<update_rate>`, so Gazebo rendered the
camera on every physics step (~940 Hz here) while the plugin published at 30 Hz.
Added `<update_rate>30</update_rate>`.

## Known behaviour, not a bug to patch

OpenVINS publishes **no odometry while the robot is stationary**:
`do_feature_propagate_update()` returns early on a ZUPT update, before
`timelastupdate = message.timestamp`, and `initialized()` requires
`timelastupdate != -1`. Any consumer (e.g. `live_mapper`, `vio_odom_bridge`) sees
nothing until the robot moves fast enough that ZUPT stops firing. Drive the robot
when testing, and do not read "no /odom_world" as a broken bridge.
