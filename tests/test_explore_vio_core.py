#!/usr/bin/env python3
"""Unit tests for ExplorerCore -- the coverage state machine in explore_vio.py.

The property that matters most is that the machine NEVER commands a standstill.
OpenVINS stops publishing odometry while the robot is not moving, so a command
of (0, 0) is not a neutral action here: it freezes the map too.

Run:  python3 tests/test_explore_vio_core.py
"""
import importlib.util
import math
import os
import sys

MODULE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'openvins', 'ov_msckf', 'scripts', 'explore_vio.py')

failures = []


def load():
    spec = importlib.util.spec_from_file_location('ev', MODULE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # module-level imports are stdlib only
    return mod


def report(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ''))
    if not ok:
        failures.append(name)


def main():
    ev = load()
    cfg = dict(ev.DEFAULT_CFG)
    dt = 1.0 / 20.0

    print(f"module: {MODULE}\n")

    # ---------------------------------------------------------------- 1
    print("[1] FORWARD -> TURN once the distance is covered")
    core = ev.ExplorerCore(cfg)
    pose = [0.0, 0.0, 0.0]
    states = []
    for i in range(400):
        # pretend the robot really moves at the commanded speed
        v, w = core.update(dt, tuple(pose), True)
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
        states.append(core.state)
        if core.state == ev.ExplorerCore.TURN:
            break
    report("reaches TURN", core.state == ev.ExplorerCore.TURN,
           f"after {i*dt:.1f}s, travelled {math.hypot(pose[0], pose[1]):.2f}m")
    report("covered the requested distance",
           math.hypot(pose[0], pose[1]) >= cfg['forward_dist'] * 0.95,
           f"{math.hypot(pose[0], pose[1]):.2f} >= {cfg['forward_dist']}")

    # ---------------------------------------------------------------- 2
    print("\n[2] FORWARD -> TURN on timeout when odometry reports no progress")
    print("    (the wedged case: wheels turn, VIO sees nothing)")
    core = ev.ExplorerCore(cfg)
    frozen = (0.0, 0.0, 0.0)
    moved = False
    t = 0.0
    while t < cfg['forward_timeout'] * 2:
        v, w = core.update(dt, frozen, True)
        t += dt
        if core.state == ev.ExplorerCore.TURN:
            moved = True
            break
    report("times out into TURN", moved, f"after {t:.1f}s")
    report("timeout is close to the configured value",
           abs(t - cfg['forward_timeout']) < 0.3, f"{t:.2f} vs {cfg['forward_timeout']}")

    # ---------------------------------------------------------------- 3
    print("\n[3] TURN -> FORWARD once the angle is turned")
    core = ev.ExplorerCore(cfg)
    turn_runtime = None
    pose = [0.0, 0.0, 0.0]
    core._enter(ev.ExplorerCore.TURN, tuple(pose))
    for i in range(400):
        v, w = core.update(dt, tuple(pose), True)
        pose[2] += w * dt
        if core.state == ev.ExplorerCore.FORWARD:
            turn_runtime = i * dt
            break
    report("reaches FORWARD", core.state == ev.ExplorerCore.FORWARD,
           f"after {turn_runtime:.1f}s, turned {math.degrees(abs(pose[2])):.0f} deg")

    # ---------------------------------------------------------------- 4
    print("\n[4] NEVER commands a standstill (the deadlock property)")
    core = ev.ExplorerCore(cfg)
    pose = [0.0, 0.0, 0.0]
    zero_cmds = 0
    for i in range(4000):                      # 200 s of simulated operation
        v, w = core.update(dt, tuple(pose), True)
        if abs(v) < 1e-9 and abs(w) < 1e-9:
            zero_cmds += 1
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
    report("zero (v, w) commands", zero_cmds == 0, f"{zero_cmds} out of 4000 ticks")

    # ---------------------------------------------------------------- 5
    print("\n[5] Stale odometry triggers RECOVER, which reverses")
    core = ev.ExplorerCore(cfg)
    pose = [0.0, 0.0, 0.0]
    for _ in range(10):
        core.update(dt, tuple(pose), True)
    core.lose_odom(tuple(pose))
    report("enters RECOVER", core.state == ev.ExplorerCore.RECOVER)
    v, w = core.update(dt, tuple(pose), False)
    report("reverses while recovering", v < 0, f"linear.x = {v}")

    # ---------------------------------------------------------------- 6
    print("\n[6] RECOVER hands back to TURN, then exploring resumes")
    core = ev.ExplorerCore(cfg)
    core._enter(ev.ExplorerCore.RECOVER, (0.0, 0.0, 0.0))
    t = 0.0
    seq = [core.state]
    for _ in range(2000):
        v, w = core.update(dt, (0.0, 0.0, 0.0), True)
        t += dt
        if core.state != seq[-1]:
            seq.append(core.state)
        if len(seq) >= 3:
            break
    report("RECOVER -> TURN -> FORWARD",
           seq[:3] == [ev.ExplorerCore.RECOVER, ev.ExplorerCore.TURN, ev.ExplorerCore.FORWARD],
           ' -> '.join(seq))

    # ---------------------------------------------------------------- 7
    print("\n[7] Turn direction alternates (the pattern does not spiral)")
    core = ev.ExplorerCore(cfg)
    dirs = []
    pose = [0.0, 0.0, 0.0]
    for _ in range(3000):
        v, w = core.update(dt, tuple(pose), True)
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
        if core.state == ev.ExplorerCore.TURN and (not dirs or dirs[-1] != core.turn_dir):
            dirs.append(core.turn_dir)
    report("both directions used", len(set(dirs)) >= 2, f"directions seen: {dirs[:4]}")

    print()
    if failures:
        print(f"FAILED: {failures}")
        sys.exit(1)
    print("ALL TESTS PASSED")


if __name__ == '__main__':
    main()
