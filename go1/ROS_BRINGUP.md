# Go1 ROS bring-up — procedure and acceptance criteria

Prepared offline 2026-10-11 while the robot was charging. **Nothing here has been
run against the robot yet.** Every claim about what the code does was read out of
the source or disassembled from the shipped library, not assumed.

---

## 0a. UPDATE 2026-10-11 — the robot runs its own ROS master, so step 1 does not need `ros_udp` at all

Discovered while ssh'd in. **`192.168.123.161:11311` is a live ROS master** with the robot's
own nodes registered. Verified from the VM: no local roscore, no local 11311 listener — the
whole graph is on the robot. Measured topics:

| Topic | Type | Rate |
|---|---|---|
| `/ros2udp/odom` | `nav_msgs/Odometry` | **200 Hz** |
| `/camera1/point_cloud_face` | `PointCloud2` | 12.9 Hz |
| `/camera3/point_cloud_left` | `PointCloud2` | 11.4 Hz |
| `/range_front` | `sensor_msgs/Range` | 50 Hz |
| `/tf` | `tf2_msgs/TFMessage` | 2299 Hz |
| `/joint_states` | `sensor_msgs/JointState` | no data at the time of measuring |

Nodes: `/camera1`, `/camera3`, `/camera4`, `/camera5`, `/node_lcm`,
`/node_obstacle_avoidance`, `/node_obstacle_traverse`, `/ros2udp_motion_mode_adv`,
`/ukd_triple_*`.

**This removes the problem in §0.** Subscribing to a master someone else runs is genuinely
passive — no `ros_udp`, therefore no 500 Hz `mode = 0` command, therefore the robot does not
stand up and nothing is commanded at all. §0 is retained below because it still applies if
we ever do run `unitree_legged_real`'s own bridge.

Two caveats found at the same time:

- **The camera topics are point clouds, not images.** `/camera1/point_cloud_face` carries
  **251 points** with fields `x, y, z, rgb` in frame `trunk`. These are the downsampled
  obstacle clouds the avoidance nodes consume, not a dense depth image. There are **no image
  topics on this master**, so the H.264/UDP path is still needed for anything VIO-shaped.
- `frame_id` is `uodom` with child `base_link` on the odometry, and the stamp is a **current**
  Unix time (~2026-10-11), i.e. `.161`'s clock is roughly right even though the three Jetsons'
  are not. See `TIMESYNC.md`.

### Revised step 1 — genuinely read-only

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.123.161:11311
export ROS_IP=192.168.123.101        # so the robot can reach us back
rostopic list
rostopic hz  /ros2udp/odom
rostopic echo -n1 /ros2udp/odom
rostopic hz  /range_front
```

Nothing here publishes. The robot is already running; we are only listening. Acceptance is
the same shape as below: `/ros2udp/odom` ≈ 200 Hz sustained, sane field values, and `/tf`
populated. **This is the test to run first**; §2's `ros_udp` path is only needed if and when
we want the SDK's own command interface.

---

## 0. Read this first — the stack is NOT passively read-only

The brief was "step 1 read-only, subscribe to the state topics, send no commands".
**The shipped package cannot do that, and it matters.**

`unitree_legged_real/src/exe/ros_udp.cpp`:

```cpp
HighCmd high_cmd = {0};              // zero-initialised
high_udp.InitCmdData(high_cmd);      // fills head[0..1] and levelFlag only
...
void highUdpSend() { high_udp.SetSend(high_cmd); high_udp.Send(); }
LoopFunc loop_udpSend("high_udp_send", 0.002, 3, ...);   // 500 Hz
loop_udpSend.start();
```

`Send()` runs from startup at **500 Hz**, unconditionally. Disassembling
`UDP::InitCmdData(HighCmd&)` from `lib/cpp/amd64/libunitree_legged_sdk.a` shows it
touches only three bytes:

```asm
movzbl (%rax),%edx   ; mov %dl,(%rsi)        # head[0]
movzbl 0x1(%rax),%eax; mov %al,0x1(%rsi)     # head[1]
movb $0xee,0x2(%rsi)                          # levelFlag = 0xEE (HIGHLEVEL)
ret
```

It does **not** set `mode`. So `high_cmd.mode` stays 0, and in the SDK's own words
mode 0 is *"idle, default stand"*.

**Consequence:** the moment `ros_udp highlevel` starts, the robot is being told
"idle / default stand" 500 times a second. It sends no velocity and no position,
so the robot will not walk — but a Go1 lying down will very likely **stand up**.
That is a physical motion and needs to be expected, not discovered.

There is no "do nothing" mode in the protocol to send instead. Treat step 1 as
*"no motion commands"*, not *"no commands"*, and brief whoever is watching
accordingly.

---

## 1. What is built, and where

```
~/go1_ws/                                  catkin workspace (catkin_tools)
  src/unitree_ros_to_real/                 cloned; SDK submodule initialised
    unitree_legged_sdk/                    v3.8.6, Go1-only, catkin package
    unitree_legged_msgs/                   HighState, LowState, IMU, ...
    unitree_legged_real/                   the bridge + examples
  devel/lib/unitree_legged_real/
    ros_udp              the bridge                   <-- the one that matters
    state_sub            prints state, sends nothing <-- the safe watcher
    twist_sub            /cmd_vel -> robot
    control_via_keyboard keyboard teleop
    ros_example_walk     motion example  \  NOT for step 1
    ros_example_position motion example  /  or step 2
```

Build verified: `catkin build unitree_legged_real` → *4 packages succeeded*, 23.5 s.

> Note there are **two** copies of the SDK on disk. The one that matters is the
> submodule inside `unitree_ros_to_real` (v3.8.6, has `package.xml`). The loose
> `~/go1_ws/src/unitree_legged_sdk` is a v3.8.0 clone with no `package.xml`, so
> catkin ignores it — but do not point a build at it by mistake.

Environment:

```bash
source ~/VINS-Nav/go1/setup_go1_env.sh
```

---

## 2. Step 1 — state stream only (no motion commands)

### Before starting

- Robot **on the ground**, flat clear area, ≥2 m clearance on all sides
- Someone standing by the power switch; **hand near it, not on the robot**
- Make sure **nothing else is talking to the robot** — the control board accepts a
  single client. `ps aux | grep -i unitree` on this VM should show only what we start
- Confirm the network first (the Codex will have done this): `ping 192.168.123.10`
  and `ip route get 192.168.123.10` → `dev ens37`

### Commands

```bash
source ~/VINS-Nav/go1/setup_go1_env.sh
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

`real.launch` is thin — it just runs `ros_udp` with that argument. In a second
terminal, watch the stream without adding any traffic:

```bash
source ~/VINS-Nav/go1/setup_go1_env.sh
rosrun unitree_legged_real state_sub
```

Then measure it properly rather than eyeballing `state_sub`'s printf spam:

```bash
rostopic hz  /high_state          # expect ~500 Hz
rostopic echo -n1 /high_state/imu
```

### Expected

- `ros_udp` prints nothing alarming; it is a 2 ms send + 2 ms receive loop
- `/high_state` publishes at roughly **500 Hz** (the loop period is 0.002 s)
- `/high_state/imu` carries a quaternion, gyro, accel and rpy that are *plausible*:
  near-unit quaternion, gyro ≈ 0 when still, accel magnitude ≈ **9.8–9.9 m/s²**
- Battery fields in the message are sane (not zero, not nonsense)
- **The robot stands up**, or at least stiffens, within a few seconds of start

### Acceptance

1. `/high_state` ≥ 400 Hz sustained for 60 s
2. `|accel|` in 9.5–10.1 m/s² while stationary — a wrong gravity magnitude means
   the frame or the units are not what we think, which would poison any VIO work
3. Quaternion stays normalised (|q| within 1e-3 of 1) over 60 s
4. No packet loss storms: the SDK tracks a `lose_recv` counter; a few are fine,
   sustained loss is not
5. Killing `ros_udp` returns the robot to whatever it does without a client —
   confirm it does not keep walking

### If it fails, check in this order

| Symptom | Look at |
|---|---|
| No `/high_state` at all | is `ros_udp` actually running? is ROS_MASTER reachable? |
| `ros_udp` starts but nothing arrives | `ping 192.168.123.161`; confirm the route is `ens37` and not the NAT gateway; check the firewall on both ends |
| Data arrives but the robot ignores us | another client already holds the connection (the board allows one) — find and stop it |
| Robot does not stand | is it in a mode that ignores high-level commands? was the battery seated properly? (see the field notes in `../docs/unitree_go1.md` §8.1) |
| Rate far below 500 Hz | CPU starvation or a saturated link; `top`, and check `rostopic hz` on a second topic at the same time |
| Works, then stops | battery sag or Wi-Fi/Ethernet flap; watch the voltage in the state message |

---

## 3. Step 2 — motion, only after step 1 is clean

Not to be started without saying so first. Robot on the ground, clear space, a
person watching, hand on the power switch.

The safe first motion is **velocity mode with a very small command**, not a
walking example. `ros_example_walk` and `ros_example_position` are open-loop
scripted motions with no feedback — fine on a bench, a bad first command.

Sketch:

```bash
# terminal 1
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
# terminal 2 — publish a single zero-velocity velocity-mode command and watch
rostopic pub -r 2 /high_cmd unitree_legged_msgs/HighCmd \
  "{mode: 2, gaitType: 1, velocity: [0.0, 0.0], yawSpeed: 0.0}"
```

`mode: 2` = target-velocity walking; zero velocity should hold position. Confirm
the robot is stable under it before raising the value in small steps, and confirm
it stops when the publisher stops (it will not — `ros_udp` keeps sending the last
command, so plan the stop: republish zeros, then Ctrl-C).

---

## 4. Coordination still outstanding

- **Camera nodes**: the user has agreed to killing `camerarosnode` / `imageai` —
  **processes only, no file deletion, no disabling autostart**. To be done at the
  camera stage, not now. Before doing it, record the full command lines of the
  processes being killed, and prepare a restart script that does not depend on a
  reboot. Not yet written.
- **Timestamps**: the Jetsons have no RTC, and the H.264/RTP stream shows no
  obvious time base. Both matter before any of this can feed a VIO estimator, and
  neither is solved.
