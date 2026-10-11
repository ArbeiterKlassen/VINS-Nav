#!/usr/bin/env python3
"""
explore_vio.py -- drive a coverage pattern using only VIO odometry.

WHY THIS EXISTS
---------------
The scripted explorers in `scripts/` (full_explore.py, bounce_explore.py, ...) do
not work for the live VINS-Nav pipeline:

  * `full_explore.py` is open-loop. It drives fixed durations, so it wedges itself
    against a wall and then sits there.
  * `bounce_explore.py` / `laser_circle.py` need `/scan`, which stops publishing
    after a few minutes in Gazebo on this machine.

And a stationary robot is not a neutral failure here: **OpenVINS stops publishing
odometry entirely while the robot is not moving** (the ZUPT update returns early
before `timelastupdate` is set -- see docs/openvins_local_patches.md). So a wedged
robot freezes the map as well as the motion. The explorer must keep the robot
moving, and must not depend on odometry *while stuck*.

DESIGN
------
A three-state machine, run at a fixed rate:

  FORWARD  drive straight until the odometry says `forward_dist` metres have been
           covered, or the timeout expires, or odometry goes stale
  TURN     rotate in place by `turn_angle` (alternating direction), same guards
  RECOVER  reverse briefly, then hand back to TURN -- this is what unsticks the
           robot when it has driven into something

Progress is measured from odometry when it is available and falling back to
elapsed time when it is not, so a VIO dropout degrades the pattern instead of
stopping the robot. `ExplorerCore` holds no ROS state so the transitions can be
unit-tested; `ExplorerNode` is only glue.

TOPICS
------
  IN : /ov_msckf/odomimu   nav_msgs/Odometry   (global -> imu)
  OUT: /cmd_vel            geometry_msgs/Twist
"""

import math
import random

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ----------------------------------------------------------------------
# testable core -- no ROS imports, no wall clock, no I/O
# ----------------------------------------------------------------------
class ExplorerCore:
    """Coverage state machine. `update()` is pure w.r.t. everything but `self`."""

    FORWARD, TURN, RECOVER = 'FORWARD', 'TURN', 'RECOVER'

    def __init__(self, cfg):
        self.cfg = cfg
        self.state = self.FORWARD
        self.state_age = 0.0          # seconds spent in the current state
        self.recover_phase = 'BACK'   # RECOVER runs BACK then hands to TURN
        self.turn_dir = cfg['turn_dir']
        self.anchor = None            # (x, y, yaw) captured at state entry
        self.rng = random.Random(cfg.get('seed', None))

    # -- helpers -------------------------------------------------------
    def _bearing_delta(self, yaw, ref):
        """Shortest signed angle from `ref` to `yaw`, in (-pi, pi]."""
        return math.atan2(math.sin(yaw - ref), math.cos(yaw - ref))

    def _enter(self, state, pose):
        self.state = state
        self.state_age = 0.0
        self.anchor = pose

    # -- main step -----------------------------------------------------
    def update(self, dt, pose, odom_ok):
        """Advance the machine by `dt` seconds and return (linear_x, angular_z).

        `pose` is (x, y, yaw) or None; `odom_ok` says whether that pose is fresh.
        """
        cfg = self.cfg
        self.state_age += dt

        if self.state == self.FORWARD:
            if self.anchor is None:
                self._enter(self.FORWARD, pose)
            # done when we have covered the distance, or run out of time
            timed_out = self.state_age > cfg['forward_timeout']
            covered = False
            if odom_ok and pose is not None and self.anchor is not None:
                covered = math.hypot(pose[0] - self.anchor[0],
                                     pose[1] - self.anchor[1]) >= cfg['forward_dist']
            if covered or timed_out:
                # alternate turn direction so the pattern does not spiral
                self.turn_dir = -self.turn_dir
                self._enter(self.TURN, pose)
            return cfg['forward_speed'], 0.0

        if self.state == self.TURN:
            if self.anchor is None:
                self._enter(self.TURN, pose)
            timed_out = self.state_age > cfg['turn_timeout']
            turned = False
            if odom_ok and pose is not None and self.anchor is not None:
                turned = abs(self._bearing_delta(pose[2], self.anchor[2])) >= cfg['turn_angle']
            if turned or timed_out:
                self._enter(self.FORWARD, pose)
            return 0.0, cfg['turn_speed'] * self.turn_dir

        if self.state == self.RECOVER:
            if self.recover_phase == 'BACK':
                if self.state_age >= cfg['recover_back_time']:
                    self.recover_phase = 'TURN'
                    self._enter(self.RECOVER, pose)
                return -cfg['recover_speed'], 0.0
            # TURN phase: rotate for a fixed spell, then resume exploring
            if self.state_age >= cfg['recover_turn_time']:
                self.recover_phase = 'BACK'
                self.turn_dir = -self.turn_dir
                self._enter(self.TURN, pose)
            return 0.0, cfg['turn_speed'] * self.turn_dir

        # unreachable, but fail safe rather than freeze the robot
        self._enter(self.FORWARD, pose)
        return cfg['forward_speed'], 0.0

    def lose_odom(self, pose):
        """Odometry went stale mid-motion -- back out and try again."""
        self.recover_phase = 'BACK'
        self._enter(self.RECOVER, pose)


DEFAULT_CFG = dict(
    forward_speed=0.18,
    forward_dist=1.2,
    forward_timeout=12.0,
    turn_speed=0.7,
    turn_angle=math.radians(80.0),
    turn_timeout=6.0,
    recover_speed=0.15,
    recover_back_time=2.0,
    recover_turn_time=2.0,
    odom_timeout=1.0,
    turn_dir=1.0,
    seed=None,
)


# ----------------------------------------------------------------------
# ROS glue
# ----------------------------------------------------------------------
class ExplorerNode:
    def __init__(self):
        rospy.init_node('explore_vio')

        cfg = dict(DEFAULT_CFG)
        for k in cfg:
            cfg[k] = rospy.get_param('~' + k, cfg[k])
        self.cfg = cfg
        self.core = ExplorerCore(cfg)

        self.last_odom = None          # (x, y, yaw)
        self.last_odom_wall = None     # wall-clock time of that sample
        self.last_tick = None

        # OpenVINS needs the robot STATIONARY to complete its ZUPT initialization,
        # and it only starts publishing odometry once the robot moves -- so there
        # is no observable signal to wait on. Hold still for a fixed spell first,
        # otherwise the explorer drives from t=0 and initialization never
        # completes (observed: 0 successes, 118 "only N valid features" failures).
        self.start_delay = rospy.get_param('~start_delay', 45.0)
        self.t0 = None

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.odom_cb, queue_size=20)

        self.rate_hz = rospy.get_param('~rate', 20.0)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.tick)

        rospy.on_shutdown(self.stop)
        rospy.loginfo("explore_vio: forward %.2fm @ %.2fm/s, turn %.0f deg, recover %.1fs back",
                      cfg['forward_dist'], cfg['forward_speed'],
                      math.degrees(cfg['turn_angle']), cfg['recover_back_time'])

    # ------------------------------------------------------------------
    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom = (p.x, p.y, yaw)
        self.last_odom_wall = rospy.get_time()

    # ------------------------------------------------------------------
    def tick(self, _evt):
        now = rospy.get_time()
        if self.last_tick is None:
            self.last_tick = now
            self.t0 = now
            return
        dt = max(1e-3, min(now - self.last_tick, 0.5))   # clamp sim-time jumps
        self.last_tick = now

        # Sit still while the estimator initializes (see __init__).
        if self.t0 is not None and (now - self.t0) < self.start_delay:
            self.pub.publish(Twist())
            return

        odom_ok = (self.last_odom_wall is not None
                   and (now - self.last_odom_wall) < self.cfg['odom_timeout'])

        # Losing odometry while moving is the wedge case: the robot is pushing
        # against something and the VIO sees no motion. Back out before trying
        # the pattern again -- but only from FORWARD/TURN, and not repeatedly.
        if not odom_ok and self.core.state in (ExplorerCore.FORWARD, ExplorerCore.TURN):
            if self.core.state_age > self.cfg['odom_timeout']:
                self.core.lose_odom(self.last_odom)

        v, w = self.core.update(dt, self.last_odom, odom_ok)
        m = Twist()
        m.linear.x = v
        m.angular.z = w
        self.pub.publish(m)

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ExplorerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
