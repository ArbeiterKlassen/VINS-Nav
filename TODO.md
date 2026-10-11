# TODO

Current priorities, in order. Context for each is in the linked docs.

## 1. ~~Make `live_mapper` height classification robust to VIO Z drift~~ — DONE 2026-10-10

**Problem.** `live_mapper.py` splits points into wall/floor by absolute height
(`wall_zmin=0.15`, `floor_zmax=0.15`). The VIO's estimated height drifts — measured
~0.25 m over a 100 s circle drive in Gazebo — which lifts the floor out of the band.
The result was `wall pts 7989/7989 (100%)` and a map with no free space at all.

**Direction.** Anchor the ground reference to the robot rather than to the world
origin, e.g. subtract the robot's current estimated Z (smoothed) before classifying,
with a slow-tracking reference so genuine floor slope is still handled. Same idea
applies to `build_map.py` for offline runs on long bags.

**Verify.** Does not need Gazebo — a bag replay is enough. Re-run the 400 s window and
check that the wall fraction stays in the 40–60% range seen on good runs rather than
saturating at 100%.

**Done.** `live_mapper.classify()` anchors the bands to `ground_z = robot_z -
imu_height`, smoothed with a 2 s EMA; `~relative_height:=false` restores the old
behaviour. Verified two ways:

- `tests/test_live_mapper_classify.py` — synthetic scene with +0.25 m of injected
  drift: absolute mode classifies **0%** of floor points correctly (reproducing the
  Gazebo failure), relative mode **100%**. The EMA rejects a 1 cm spike (moves 0.5 mm)
  but follows a 0.25 m slow drift (moves 250 mm).
- Bag regression, 400 s, same window as the earlier run: occupied 5589 → 5631
  (+0.8%), free 10535 → 10557 (+0.2%) — no degradation when there is no drift. The
  diagnostics now report the reference (`ground=-0.081 … -0.038`) and the wall
  fraction held at 49–63% throughout.

## 2. Solve the drive problem for live Gazebo testing — IN PROGRESS 2026-10-11

**Problem.** Two separate blockers:
- Open-loop drive patterns get stuck against walls. Once the robot stops, OpenVINS
  stops publishing odometry entirely (ZUPT returns early before `timelastupdate` is
  set — see `docs/openvins_local_patches.md`), so the map stops growing.
- The reactive explorers (`bounce_explore.py`, `laser_circle.py`, …) need `/scan`,
  which stops publishing after a few minutes on this VM.

**Direction.** Either fix the laser, or write an exploration node that needs no
sensors beyond what already works — e.g. drive a sequence of VIO-relative waypoints
(forward N metres, turn M degrees) with a timeout and a reverse-and-turn recovery,
so the robot is never stationary for long.

**Verify.** Robot keeps moving for 3+ minutes without getting stuck; `/ov_msckf/odomimu`
publishes continuously.

**Progress.** `explore_vio.py` written: a FORWARD / TURN / RECOVER state machine driven
by VIO odometry with a timeout fallback, so a VIO dropout degrades the pattern instead
of stopping the robot. `tests/test_explore_vio_core.py` covers seven properties, the
load-bearing one being **never commands a standstill** (0 of 4000 ticks) — a stationary
robot freezes the map, not just the motion.

One design flaw found and fixed during testing: driving from t=0 prevents the ZUPT
initialisation, because the robot is never still long enough (observed 0 successful
inits, 118 "only N valid features" failures). Added `~start_delay` (default 45 s) —
OpenVINS needs the robot stationary to initialise and only publishes once it moves, so
there is no observable signal to wait on, and a fixed hold is the honest answer.

**Still open:** the Gazebo integration run did not complete. The camera stream stops
after a few minutes on this VM, which now blocks both this item and item 3. That
degradation is worth attacking on its own.

## 3. Re-run the full live mapping test against the ground truth

**Depends on 1 and 2.**

Run a full Gazebo session, save the map, and compare it against
`maps/ground_truth_annotated.png`. The bar set by the earlier work: `live_mapper`
fed the recorded poses reached 5589 occupied / 10535 free cells and matched
`build_map.py` almost exactly — so the algorithm is proven; this run tests whether
the *live* pipeline reaches the same quality.

---

## Known issues that are not on the critical path

- **Recording drops frames.** `house_full.bag` lost ~38% of camera frames with
  intervals from 34–194 ms, which destabilises the VIO scale (CV 0.182 vs 0.029 on
  the original live run). Reducing the recording load would make bag replay
  representative of live conditions again.
- **Two localized VIO failure windows** in the original recording (t≈955–975 s and
  t≈1121–1245 s) remain unexplained; only located, not diagnosed.
- **Full-stack CPU headroom.** `pseudo_stereo` drops to ~13 Hz with RTAB-Map and
  EGO-Planner running; the research notes suggest `num_pts` 600→300–400 and
  `num_opencv_threads` 4→2–3 if it matters.
- **Closed-loop navigation** (goal → autonomous drive) has still never been run
  end-to-end.

## Hardware

- **Gazebo / TurtleBot3** — the current development platform, on a VMware VM.
- **Unitree Go1** — acquired 2026-10-10; SDK availability and hardware configuration
  under investigation, see `docs/unitree_go1.md` once written.
