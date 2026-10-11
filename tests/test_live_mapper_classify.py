#!/usr/bin/env python3
"""Unit test for LiveMapper.classify -- the wall/floor split.

Reproduces the real failure: measured in Gazebo 2026-09-09, ~0.25 m of VIO
height drift lifted the floor out of its band and classified 100% of points as
wall, leaving the map with no free space.

Run:  python3 /tmp/test_live_mapper_classify.py
"""
import importlib.util
import sys
import types

import numpy as np

import os
MODULE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', 'openvins', 'ov_msckf', 'scripts', 'live_mapper.py')


class Stamp:
    """Stand-in for rospy.Time so the test needs no ROS graph."""
    def __init__(self, t):
        self.t = t

    def to_sec(self):
        return self.t


def load_classify():
    spec = importlib.util.spec_from_file_location('lm', MODULE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.LiveMapper.classify


def make_node(classify, relative, ground_tau=2.0):
    """Bare object carrying just the attributes classify touches."""
    node = types.SimpleNamespace(
        relative_height=relative,
        imu_height=0.078,
        ground_tau=ground_tau,
        ground_z=None,
        _ground_t=None,
        wall_zmin=0.15, wall_zmax=3.0,
        floor_zmin=-0.5, floor_zmax=0.15,
    )
    return types.SimpleNamespace(classify=lambda zw, rz, st: classify(node, zw, rz, st),
                                 state=node)


# A scene: 500 floor points at height 0, 500 wall points at height 1.0.
FLOOR_H, WALL_H = 0.0, 1.0
zw = np.concatenate([np.full(500, FLOOR_H), np.full(500, WALL_H)])
is_floor_truth = np.concatenate([np.ones(500, bool), np.zeros(500, bool)])

failures = []


def check(name, node, robot_z, expect_floor_frac, expect_wall_frac, tol=0.10):
    wall, floor = node.classify(zw, robot_z, Stamp(0.0))
    ff = floor[is_floor_truth].mean()          # of the true floor points
    wf = wall[~is_floor_truth].mean()          # of the true wall points
    ok = abs(ff - expect_floor_frac) <= tol and abs(wf - expect_wall_frac) <= tol
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"        floor correctly classified: {ff*100:5.1f}%  (want ~{expect_floor_frac*100:.0f}%)")
    print(f"        wall  correctly classified: {wf*100:5.1f}%  (want ~{expect_wall_frac*100:.0f}%)")
    if not ok:
        failures.append(name)


def check_must_fail(name, node, robot_z, floor_frac_below=0.10):
    """For cases where breaking is the expected, documented behaviour."""
    _, floor = node.classify(zw, robot_z, Stamp(0.0))
    ff = floor[is_floor_truth].mean()
    ok = ff <= floor_frac_below
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print(f"        floor correctly classified: {ff*100:5.1f}%  (expected to be near 0)")
    if not ok:
        failures.append(name)


print(f"module: {MODULE}")
print(f"scene : floor at Z={FLOOR_H}, wall at Z={WALL_H}\n")

classify = load_classify()

print("[1] No drift -- robot IMU at nominal 0.078 m")
check("absolute mode", make_node(classify, False), 0.078, 1.0, 1.0)
check("relative mode", make_node(classify, True), 0.078, 1.0, 1.0)

print("\n[2] +0.25 m VIO height drift -- this is the observed failure")
# The robot has not actually risen; only the estimate is wrong. Floor points are
# still at Z=0 in the (drifted) map frame? No -- the whole map is expressed in the
# drifted estimate, so the floor lands ~0.25 m higher.
zw_drift = np.concatenate([np.full(500, FLOOR_H + 0.25), np.full(500, WALL_H + 0.25)])
zw_save = zw
zw = zw_drift
check_must_fail("absolute mode -- breaks, as observed in Gazebo",
                make_node(classify, False), 0.078 + 0.25)
check("relative mode -- survives the same drift",
      make_node(classify, True), 0.078 + 0.25, 1.0, 1.0)
zw = zw_save

print("\n[3] Ground reference follows a slow drift, not a per-frame spike")
node = make_node(classify, True, ground_tau=2.0)
node.classify(zw, 0.078, Stamp(0.0))                     # seed the reference
node.classify(zw, 0.078, Stamp(10.0))                    # 10 s, no drift
g_before = node.state.ground_z
node.classify(zw, 0.078 + 0.01, Stamp(10.1))             # 1 cm spike in 0.1 s
g_spike = node.state.ground_z
node.classify(zw, 0.078 + 0.25, Stamp(110.0))            # 0.25 m over 100 s
g_after = node.state.ground_z
print(f"        ground_z after settle : {g_before:.4f}")
print(f"        ground_z after 1cm spike: {g_spike:.4f}  (moved {abs(g_spike-g_before)*1000:.1f} mm)")
print(f"        ground_z after slow drift: {g_after:.4f}  (moved {abs(g_after-g_before)*1000:.1f} mm)")
spike_ok = abs(g_spike - g_before) < 0.002
drift_ok = abs(g_after - (0.078 + 0.25 - 0.078)) < 0.01
print(f"  {'PASS' if spike_ok else 'FAIL'}  spike rejected")
print(f"  {'PASS' if drift_ok else 'FAIL'}  drift followed")
if not spike_ok:
    failures.append('spike rejected')
if not drift_ok:
    failures.append('drift followed')

print()
if failures:
    print(f"FAILED: {failures}")
    sys.exit(1)
print("ALL TESTS PASSED")
