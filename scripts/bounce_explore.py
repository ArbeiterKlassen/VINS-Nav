#!/usr/bin/env python3
"""
Bounce explorer: drive straight until blocked, bounce to new direction.
Minimal rotation = fewer artifacts. Handles narrow spaces by backing out.
"""
import rospy, time, math, random, numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BounceExplorer:
    def __init__(self):
        self.scan = None
        rospy.init_node('bounce_explore')
        rospy.Subscriber('/scan', LaserScan, self.cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def cb(self, m):
        self.scan = np.array(m.ranges)

    def _ranges(self):
        r = self.scan.copy() if self.scan is not None else np.ones(360)*10
        r[np.isinf(r)] = 10.0
        return r

    def front_clear(self, margin=0.5):
        r = self._ranges()
        n = len(r)
        front = np.concatenate([r[:n//8], r[7*n//8:]])  # front 45 deg
        return np.min(front) > margin

    def find_gap(self):
        """Find the largest angular gap in laser scan. Return center angle."""
        r = self._ranges()
        n = len(r)
        # Scan for longest run of readings > 0.6m
        gap_start, best_len, best_start = 0, 0, 0
        in_gap = False
        for i in range(2*n):  # wrap around
            idx = i % n
            if r[idx] > 0.6:
                if not in_gap:
                    gap_start = idx; in_gap = True
                length = (idx - gap_start) % n
                if length > best_len:
                    best_len = length; best_start = gap_start
            else:
                in_gap = False
        ang = (best_start + best_len//2) % n
        return (ang / n - 0.5) * 2 * math.pi

    def narrow_space(self):
        """Check if we're boxed in (multiple sides blocked)."""
        r = self._ranges()
        n = len(r)
        quarters = [r[i*n//4:(i+1)*n//4] for i in range(4)]
        blocked_count = sum(1 for q in quarters if np.min(q) < 0.5)
        return blocked_count >= 3

    def _min_front_angle(self):
        """Return angle (rad) of nearest obstacle in front half."""
        r = self._ranges()
        n = len(r)
        front = np.concatenate([r[:n//4], r[3*n//4:]])
        idx = np.argmin(front)
        total = len(front)
        return (idx / total - 0.5) * math.pi  # map to [-pi/2, pi/2]

    def _is_corridor(self):
        """Detect if we're in a corridor: long clear path ahead and behind, walls on sides."""
        r = self._ranges()
        n = len(r)
        front = np.concatenate([r[:n//8], r[7*n//8:]])  # front 45 deg
        back = r[3*n//8:5*n//8]  # back 45 deg
        left = r[n//4:3*n//8]    # left side
        right = r[5*n//8:3*n//4] # right side
        return (np.min(front) > 1.5 or np.min(back) > 1.5) and \
               np.min(left) < 0.6 and np.min(right) < 0.6

    def run(self):
        rospy.sleep(3)
        print("Bounce Explorer started")
        # Scan and face clearest direction first
        if not self.front_clear(1.0):
            gap = self.find_gap()
            print("Start blocked, facing gap at %.0f deg" % (gap*57.3))
            self._drive(0, 0.5 if gap > 0 else -0.5, 2.0)
        heading = 0.0
        t_stuck = 0

        just_reversed = False
        for step in range(500):  # max ~250s
            r = self._ranges()
            n = len(r)
            # Hysteresis: require more clearance right after reversing
            margin = 0.7 if just_reversed else 0.45
            front_blocked = self.front_clear(margin) == False
            just_reversed = False
            narrow = self.narrow_space()

            msg = Twist()

            if narrow:
                # Boxed in: reverse out carefully
                print("[%d] NARROW: backing out" % step)
                self._drive(-0.08, 0, 1.5)
                just_reversed = True
                gap = self.find_gap()
                heading = gap
                self._drive(0, heading * 0.3, 1.5)
                t_stuck = 0
            elif not front_blocked:
                # Clear ahead: drive forward, small random direction changes
                t_stuck = max(0, t_stuck - 2)  # slowly decay stuck counter
                if random.random() < 0.02:
                    heading = random.uniform(-0.2, 0.2)
                msg.linear.x = 0.15
                msg.angular.z = heading
            else:
                # Blocked ahead: find gap and turn
                t_stuck += 1
                gap = self.find_gap()
                turn_dir = 1.0 if gap > 0 else -1.0

                if t_stuck > 20:  # stuck for 2s
                    back_dist = 2.0 if t_stuck > 60 else 1.0
                    print("[%d] STUCK x%d: backing %.1fm" % (step, t_stuck, back_dist))
                    self._drive(-0.12, 0, back_dist)
                    just_reversed = True
                    # Turn to a safe direction
                    if self._is_corridor():
                        gap = self.find_gap()
                        print("  corridor: facing gap at %.0f deg" % (gap*57.3))
                        self._drive(0, 0.5 if gap > 0 else -0.5, 2.0)
                    else:
                        # Fixed 2s turn ~57 deg, direction away from nearest obstacle
                        turn_dir = 1 if self._min_front_angle() > 0 else -1
                        print("  escape turn dir=%d 2s" % turn_dir)
                        self._drive(0, 0.5 * turn_dir, 2.0)
                    t_stuck = 0
                    # Verify clear before proceeding
                    if self.front_clear(0.5):
                        break
                else:
                    # Normal blocked: small turn toward gap
                    msg.angular.z = turn_dir * min(abs(gap), 0.5)
                    msg.linear.x = 0.0

            self.pub.publish(msg)
            self.rate.sleep()

        self.pub.publish(Twist())
        print("Bounce Explorer done (%d steps)" % step)

    def _drive(self, lin, ang, dur):
        msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    BounceExplorer().run()
