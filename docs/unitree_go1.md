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

## 6. Stereo adaptation — what existing projects do

Researched 2026-10-10, looking for a proven configuration to copy rather than invent.

### 6.1 The camera-model mismatch

Verified against the local OpenVINS source: `ov_core/src/cam/` contains exactly **two**
models, `CamRadtan` and `CamEqui`, and the current config uses
`distortion_model: radtan`.

The Go1's factory calibration is a third thing. `UnitreecameraSDK`'s
`example_getCalibParamsFile` writes `output_camCalibParams.yaml` containing per-eye
**Xi**, IntrinsicMatrix and DistortionCoefficients plus a stereo rotation and baseline —
that is the **Mei unified camera model**, whose `xi` parameter has no counterpart in
either OpenVINS model. So the factory numbers cannot be dropped straight into the
estimator.

Two ways out:

- **Rectify to pinhole and use `radtan`.** `UnitreecameraSDK` already supports a
  `PERSPECTIVE` projection alongside the native `FISHEYE/LONGLAT` one, so rectified
  pinhole frames are available with distortion set to zero. This is the lower-effort
  path and is exactly what the reference project below does.
- **Re-calibrate with Kalibr** using `pinhole-equi`, which OpenVINS's `CamEqui`
  explicitly matches. More work (AprilGrid target, 2–5 Hz bags, both cameras plus
  IMU), but it gives a proper `cam0`/`cam1`/IMU chain rather than a rectified proxy.

### 6.2 The reference project — and the surprise in it

[Cerberus](https://github.com/ShuoYangRobotics/Cerberus) (Shuo Yang et al., RA-L 2022 +
ICRA 2023) is the closest existing work: **visual-inertial-leg odometry for the Unitree
A1 and Go1**, built on VINS-Fusion with an added contact-preintegration term, reporting
**<1 % position drift**. It ships Go1 configs, launch files and demo bags, and is the
best thing to read before writing any Go1 VIO code.

But its Go1 configuration does **not** use the onboard fisheye. `hardware_go1_vilo_config.yaml`
points at:

```yaml
imu_topic:    "/hardware_go1/imu"
image0_topic: "/camera_forward/infra1/image_rect_raw"
image1_topic: "/camera_forward/infra2/image_rect_raw"
leg_topic:    "/hardware_go1/joint_foot"
robot_type:   "go1"
```

and the camera yaml is `model_type: PINHOLE`, `424×240`, all distortion coefficients
zero, `fx = fy = 211.879`, `cx = 212.05`, `cy = 120.70`. Those are **RealSense D435
infra-stream numbers** (`infra1`/`infra2`, `_optical_frame`), i.e. an **external
forward-facing D435** bolted on, with the images already rectified. The proven
low-drift Go1 configuration therefore sidesteps the fisheye problem entirely.

### 6.3 What this means for the choice

| Path | Proven? | Effort | Notes |
|------|---------|--------|-------|
| Add a forward RealSense D435 | ✅ Cerberus, <1 % drift | Low | It also gives **depth**, so the existing pseudo-stereo front end transfers as-is — only recalibration is needed |
| Onboard fisheye, rectified to pinhole | ⚠️ no reference config found | Medium | Needs per-Jetson camera access, OpenCV 4.1.1 pin, and a rectification step |
| Onboard fisheye, Kalibr `pinhole-equi` | ⚠️ no reference config found | High | Cleanest model fit for `CamEqui`, but full recalibration |

**Recommendation:** if a D435 is available, take it. It is the only option with a
published working configuration on this exact robot, and — because it restores the
RGB-D input that pseudo-stereo was designed around — the VINS-Nav front end needs no
structural change at all. Chasing the onboard fisheye is interesting but would be new
work with no reference to copy.

### 6.4 The part that is genuinely new for a legged robot

Cerberus's actual contribution is worth understanding even if the camera question is
settled: when a foot is on the ground, the leg kinematics give a direct **body velocity**
measurement, and folding that in as a *contact preintegration* factor is what takes drift
below 1 %. `contact_sensor_type: 2` in their config means the foot-force sensors are used
to decide contact.

The Go1 exposes exactly what this needs — `LowState.footForce` / `footForceEst` and the
per-joint states — so this is a real, available improvement over what a purely
visual-inertial system can do on a legged platform. It is also the reason a Go1 VIO
should not simply be the TurtleBot3 stack with different topics.

## 7. Network reachability — confirmed from the VM

The Go1 is cabled to the **host machine**, not to this Ubuntu VM, so the first question
was whether the VM can reach it at all. Measured 2026-10-10:

| Target | Result |
|--------|--------|
| `192.168.123.10` (motion control board) | ✅ reachable |
| `192.168.123.161` (sport-mode service) | ✅ reachable |
| `192.168.12.1` (Raspberry Pi, wireless side) | ❌ not reachable — expected |

The route is not a coincidence: `ip route get 192.168.123.10` resolves via
`192.168.38.2` (the VMware NAT gateway, i.e. the host), and the ping shows
**2.5 ms average, 0.4 ms jitter** — a real device answering, not a NAT artefact.
So the host is forwarding for the VM and **no network reconfiguration is needed**.

Caveats worth knowing before sending anything to the robot:

- The SDK is **UDP**. NAT handles replies to the source port fine, but the robot's
  controller accepts **one client at a time**, so if the host is also running
  something that talks to it, the two will fight. Make sure nothing else on the host
  is connected.
- Nothing has been sent to the robot yet. Reachability was checked with `ping` only;
  a quadruped that moves unexpectedly is a safety matter, so the first actual command
  should be a **stand / damping** mode with the robot on the ground and clear of
  obstacles, and idle (`HighCmd.mode = 0`) to stop.

## 8. Not yet verified — to check when the robot is in hand

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

## 8. Field notes — 2026-10-11 (robot on the bench)

All facts below verified on the live unit.

### 8.1 "Boot failure" was a missing battery box

- Symptom: after powering on the battery (short press, then long press ≥ 2 s) the LED ran the
  normal sequence (sweep + level display) and went dark after ~2-3 s. No sound, no motion, no
  head LED; the robot never stood up.
- Root cause: the battery box (the carrier that seats the pack into the robot) had been taken
  out and lost. The pack was never actually mated with the robot bus, so the battery saw no
  load and shut itself down. The manual documents this: load-detection protection powers the
  battery off ~3 s after power-on when no load is connected.
- Fix: seat the pack in a proper battery box and insert it. Everything works.
- Rules of thumb: battery LED dark ~3 s after power-on + total silence = no load = a power-path
  problem (seating / box / contacts), never a software problem — a bricked computer still draws
  current. A battery turned on outside the robot also powers off in ~3 s by design.

### 8.2 Wired access from the host PC

- Cable: host gigabit NIC → robot gigabit Ethernet port (user manual appendix, port 7).
- Host NIC: static `192.168.123.100/24`, no gateway (Windows:
  `netsh interface ipv4 set address "以太网" static 192.168.123.100 255.255.255.0`; revert with
  `netsh interface ipv4 set address "以太网" dhcp`).
- Live endpoints (ping / ARP verified):

  | Endpoint | Address | Notes |
  |---|---|---|
  | Motion control board | `192.168.123.10` | ping ok |
  | Sport-mode service | `192.168.123.161` | ping ok; UDP 8082 |
  | Head Nano | `192.168.123.13` | ssh ok — `unitree` / `123` (factory default) |
  | Body Nano | `192.168.123.14` | ssh ok — `unitree` / `123` |
  | Main computer | `192.168.123.15` | Xavier NX, hostname `nx` — ssh ok — `unitree` / `123` |

- This answers the open question in section 7: `192.168.123.x` is directly reachable over the
  wired port.
- Passwordless ssh from the host PC was set up on this date (host key added to each machine's
  `authorized_keys`).
- Alternate path: the robot's WiFi hotspot `UnitreeRoboticsGo1-XXX` (password `00000000`)
  reaches the wireless-side Pi at `192.168.12.1`.

### 8.3 Onboard survey

| Machine | Board | Hostname | Root disk | `/dev/video*` | Autostart camera nodes |
|---|---|---|---|---|---|
| `.13` | Jetson Nano 4GB | `unitree-desktop` | 14G, 81% used | 2 (front + chin) | `example_point` + `point_cloud_node` running |
| `.14` | Jetson Nano 4GB | `unitree-desktop` | 14G, 57% used | 2 (left + right) | `point_cloud_node` x2 running |
| `.15` | Jetson Xavier NX | `nx` | 110G NVMe, 20% used | 1 (belly) | `point_cloud_node` running |

- All three: Ubuntu 18.04.5, kernel `4.9.201-tegra`, boot with the robot.
- `~/Unitree/autostart` is intact and starts at login: updateDependencies, ipconfig,
  gencamparams, camerarosnode, 03persontrack, imageai, faceLightServer, slamDetector, wsaudio,
  faceLightMqtt. The camera nodes hold `/dev/video*` — kill them before using
  `UnitreeCameraSDK` directly (see section 6).
- SDK trees present: `~/Unitree/sdk/{UnitreeCameraSdk,faceLightSDK_Nano,ultraSoundSDK_Nano}`.
- No RTC on these boards: clocks jump back to the image build date (2018-01-29 on `.13`).
  Do not trust file mtimes.
- `.13` root disk is 81% full (2.6G free) — be careful before recording or building there.
