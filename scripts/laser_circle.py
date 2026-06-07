#!/usr/bin/env python3
"""Laser-guided circling: wall-follow to loop around nearby obstacles."""
import rospy, time, math, numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserCircle:
    def __init__(self):
        self.scan = None
        rospy.init_node('laser_circle')
        rospy.Subscriber('/scan', LaserScan, self.cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def cb(self, m):
        self.scan = np.array(m.ranges)

    def blocked(self, threshold=0.45):
        if self.scan is None: return True
        r = self.scan.copy(); r[np.isinf(r)] = 10.0
        n = len(r)
        front = np.concatenate([r[:n//6], r[5*n//6:]])  # front 60 deg
        return np.min(front) < threshold

    def right_dist(self):
        """Distance to obstacle on the right side (for wall-following)."""
        if self.scan is None: return 1.0
        r = self.scan.copy(); r[np.isinf(r)] = 10.0
        n = len(r)
        # Right side: indices roughly 5n/6 to n (rear-right) to n/3 to n/2 (front-right)
        right = r[3*n//4:7*n//8]  # ~45 deg arc on right
        return np.median(right)

    def best_turn(self):
        """Find clearest direction."""
        if self.scan is None: return 0
        r = self.scan.copy(); r[np.isinf(r)] = 10.0
        n = len(r)
        best_ang, best_d = 0, 0
        for i in range(8):
            sector = r[i*n//8:(i+1)*n//8]
            d = np.min(sector)
            if d > best_d:
                best_d = d
                best_ang = ((i+0.5)/8 - 0.5) * 2 * math.pi
        return best_ang

    def run(self):
        rospy.sleep(3)
        print("Laser circle started - wall following on right side")
        t0 = time.time()
        last_turn_time = 0

        while not rospy.is_shutdown() and time.time() - t0 < 180:
            msg = Twist()
            right_d = self.right_dist()
            blocked = self.blocked()

            if blocked:
                # Reverse a bit
                print("BLOCKED! reverse...")
                self._drive(-0.1, 0, 0.8)
                # Incremental turn toward clearest direction until front is clear
                target_ang = self.best_turn()
                turn_dir = 1.0 if target_ang > 0 else -1.0
                print("  turning toward %.0f deg" % (target_ang * 57.3))
                for _ in range(20):  # max 20 small steps = ~3s
                    if not self.blocked(threshold=0.5):
                        break
                    msg_t = Twist(); msg_t.angular.z = 0.4 * turn_dir
                    self.pub.publish(msg_t); self.rate.sleep()
                last_turn_time = time.time()
            elif right_d < 0.4:
                # Too close to right wall: turn left away
                msg.linear.x = 0.10
                msg.angular.z = 0.3
            elif right_d > 0.8:
                # No wall on right: go straight, slight right to find wall
                msg.linear.x = 0.15
                msg.angular.z = -0.1  # gentle, won't cause circling
            else:
                # Good distance: follow wall with gentle correction
                msg.linear.x = 0.15
                msg.angular.z = (0.6 - right_d) * 0.5

            self.pub.publish(msg)
            self.rate.sleep()

        self.pub.publish(Twist())
        print("Laser circle done")

    def _drive(self, lin, ang, dur):
        msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    LaserCircle().run()
