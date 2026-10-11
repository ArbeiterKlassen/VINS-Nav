# Unitree Go1 — SDK availability and hardware configuration

Research done 2026-10-10, before the robot was in hand. **Everything below about the
software stack was verified by actually cloning and reading the source.** Everything
about the physical robot comes from Unitree's documentation and the SDK headers — it
has not been checked against the actual unit.

---

## 1. Which SDK applies — and which does not

The Go1 is an **older-generation** robot. Unitree maintains two unrelated SDK lines:

| SDK | Transport | Robots | Go1? |
|-----|-----------|--------|------|
| `unitree_legged_sdk` | raw UDP | Aliengo, A1, **Go1**, B1 | **yes — this one** |
| `unitree_sdk2` | CycloneDDS | Go2, B2, H1, G1 | no |

The `unitree_legged_sdk` README states it plainly:

> support robot: Go1
> not support robot: Laikago, Aliengo, A1.

**Use `unitree_legged_sdk` v3.8.x.** Older tags are split per robot — v3.5.1 and v3.3.1
also cover Go1, but anything before that targets other machines.

## 2. Downloadability — verified

All of these clone successfully from GitHub over plain HTTPS (measured, not assumed):

| Repo | Purpose | Clone |
|------|---------|-------|
| `unitreerobotics/unitree_legged_sdk` (`-b v3.8.0`) | C++/Python SDK, UDP transport | ✅ |
| `unitreerobotics/unitree_ros_to_real` | ROS 1 interface (`unitree_legged_msgs`, `unitree_legged_real`) | ✅ |
| `unitreerobotics/unitree_ros` | URDF descriptions + Gazebo models for all their robots | ✅ (one retry — first attempt died with a TLS error) |
| `aatb-ch/go1_republisher` | Publishes the onboard cameras as ROS topics | ✅ |
| `ngmor/unitree_camera` | Cameras under ROS 2 Humble | ✅ |

```bash
git clone -b v3.8.0 https://github.com/unitreerobotics/unitree_legged_sdk.git
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
git clone https://github.com/unitreerobotics/unitree_ros.git
```

Build dependencies are light: Boost ≥ 1.5.4, CMake ≥ 2.8.3, g++ ≥ 8.3.0. There is also
a Python binding (`cmake -DPYTHON_BUILD=TRUE ..`, needs msgpack + pybind11). The
examples need `sudo` because they lock memory.

## 3. Hardware configuration

### Joints

12 actuated joints, three per leg. Limits are in the SDK's `go1_const.h`, so these are
authoritative:

| Joint | Range (rad) | Range (deg) |
|-------|-------------|-------------|
| Hip | −1.047 … 1.047 | ±60 |
| Thigh | −0.663 … 2.966 | −38 … 170 |
| Calf | −2.721 … −0.837 | −156 … −48 |

### What the robot reports back (`LowState` / `HighState` in `comm.h`)

- **IMU** — `quaternion` (w,x,y,z, normalised), `gyroscope` (rad/s), `accelerometer`
  (m/s²), `rpy` (rad). The SDK adds a warning worth remembering:
  *"when under accelerated motion, the attitude of the robot calculated by IMU will drift."*
  The gyro and accel are flagged as *raw* — this is the same IMU a VIO backend would
  consume.
- **Motor state** — an array of 20 (Go1 populates 12): position, velocity, torque,
  temperature per joint.
- **Battery** — `BmsState`.
- **Foot force** — 4 sensors, plus an estimated variant.
- **`tick`** — the motion controller's own microsecond clock. Useful as a time base.
- **`wirelessRemote`** — the handset/joystick state, 40 bytes.

### Odometry and ranging (`HighState` only)

- `position` (m) — *"from own odometry in inertial frame, usually drift"*. Do not treat
  this as a localisation source; it is the same caveat as any legged odometry.
- `velocity` (m/s) — forward / side / rotate, **in the body frame**.
- `yawSpeed` (rad/s).
- `rangeObstacle` — 4 ranges.
- `footPosition2Body` / `footSpeed2Body` — per-foot Cartesian state.
- Gait state: `gaitType` (0 idle, 1 trot, 2 trot running, 3 climb stair, 4 trot
  obstacle), `footRaiseHeight`, `bodyHeight`.

### Control modes

`HighCmd.mode` selects behaviour: 0 idle, 1 force-stand, **2 target velocity walking**,
**3 target position walking**, 4 path mode (reserved), 5/6 stand down/up, 7 damping,
8 recovery stand, plus fixed actions (backflip, jumpYaw, dance…).

High-level and low-level control **cannot be switched at runtime**.

### Network

| Endpoint | Address | Purpose |
|----------|---------|---------|
| Motion control board | `192.168.123.10:8007` | low-level |
| Sport-mode service | `192.168.123.161:8082` | high-level |
| SDK local port | `8080` | outgoing |

The on-board computers sit on the same subnet: the Raspberry Pi at `192.168.12.1`
(wireless side) and the Jetsons at `192.168.123.13` / `.14` / `.15`.

### Cameras — a separate stack

The legged SDK has **no camera field at all**. The Go1 Edu's five stereo fisheye pairs
(head, chin, left, right, belly) are reached through `UnitreecameraSDK`, which means
SSHing into the specific Jetson, **killing Unitree's autostart camera processes** to
free the video device, then streaming over the network. Two known traps:

- `UnitreecameraSDK` is pinned to **OpenCV 4.1.1** and segfaults with newer versions.
  The Edu Jetsons ship with 4.1.1 in `/usr/local`, so build there rather than upgrading.
- Each Jetson owns different cameras (head Nano `.13`: front + chin; body Nano `.14`:
  left + right; main Nano/NX `.15`: belly).

## 4. ROS integration — and a version problem

`unitree_ros_to_real` is a **catkin (ROS 1)** package providing `unitree_legged_msgs`
(message definitions) and `unitree_legged_real` (the node that talks to the robot),
with both low-level and high-level examples. Clone into a catkin workspace, point its
`CMakeLists.txt` at the SDK path, `catkin_make`.

**But the surrounding packages officially target the wrong ROS version for this
machine.** From the `unitree_ros` README:

> * ROS Melodic or ROS Kinetic (has not been tested)

and `go1_republisher`'s install line is `ros-melodic-camera-info-manager`.
`ngmor/unitree_camera` is ROS 2 Humble. This workstation is **Ubuntu 20.04 + Noetic**.

So: the SDK itself and `unitree_ros_to_real` should build fine on Noetic (plain catkin,
no exotic dependencies), but the *simulation* packages and the camera republisher will
need porting — mostly swapping `ros-melodic-*` for `ros-noetic-*` and fixing any
`gazebo_ros_control` API drift.

## 5. What this means for VINS-Nav

The obvious question is whether the existing stack transfers. It mostly does, with one
structural gap:

| VINS-Nav needs | Go1 provides | Gap |
|----------------|--------------|-----|
| IMU at high rate | ✅ via `LowState.imu` (raw gyro/accel) | none |
| Camera images | ⚠️ only through `UnitreecameraSDK` on the Jetson | separate stack, OpenCV pin |
| Robot odometry | ⚠️ `HighState.velocity` (body frame) + drifting `position` | usable as a motion prior, not as a pose |
| Depth (for pseudo-stereo) | ❌ the fisheye pairs are stereo, not RGB-D | **pseudo-stereo does not transfer as-is** |

That last row matters most. Pseudo-stereo exists because there is one RGB-D camera and
no real stereo; the Go1 has **real stereo** but no depth. So the front end would move
from "warp depth into a fake right image" to "use the real fisheye pair" — which is
simpler in principle but brings fisheye distortion, the 4.1.1 OpenCV pin, and the need
to re-derive extrinsics per camera.

Also worth noting: the Go1 is a **legged** platform. EGO-Planner's trajectories assume a
differential-drive base — that layer would need a velocity-command adapter for
`HighCmd.velocity + yawSpeed` instead of `/cmd_vel`.

## 6. Not yet verified — to check when the robot is in hand

- Which variant: Air / Pro / **Edu**. The onboard computers and the cameras only exist
  on Edu (and the Jetson mix differs on "Max").
- Firmware version, and whether Sport Mode ≥ v1.36.0 (the SDK requires it for the
  high-level path).
- Whether the cameras are physically present and still have Unitree's autostart
  processes running.
- Actual IMU rate and noise characteristics — everything in VINS-Nav's config is
  calibrated for the simulated sensor, so the Allan-variance step would need redoing.
- Whether `192.168.123.x` is reachable over the wired port or only via the Pi's
  wireless bridge (the Japanese write-ups use SSH to `192.168.12.1` plus IP forwarding).
