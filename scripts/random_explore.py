#!/usr/bin/env python3
"""Random exploration with laser obstacle avoidance."""
import rospy, time, math, random, numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RandomExplorer:
    def __init__(self):
        self.scan = None
        rospy.Subscriber('/scan', LaserScan, self.cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.init_node('random_explore')
        self.rate = rospy.Rate(10)

    def cb(self, m):
        self.scan = np.array(m.ranges)

    def is_blocked(self):
        if self.scan is None: return True
        r = self.scan.copy()
        r[np.isinf(r)] = 10.0
        n = len(r)
        front = r[n//3:2*n//3]
        return np.min(front) < 0.45

    def best_direction(self):
        """Find the clearest direction (max min-range in sector)."""
        if self.scan is None: return 0
        r = self.scan.copy(); r[np.isinf(r)] = 10.0
        n = len(r)
        sectors = 12; best_ang = 0; best_dist = 0
        for i in range(sectors):
            s = r[i*n//sectors:(i+1)*n//sectors]
            d = np.min(s)
            if d > best_dist:
                best_dist = d
                best_ang = ((i+0.5)/sectors - 0.5) * 2 * math.pi
        return best_ang

    def run(self):
        time.sleep(3)
        direction = 0.3
        t0 = time.time()
        while not rospy.is_shutdown():
            msg = Twist()
            if self.is_blocked():
                direction = self.best_direction()
                msg.angular.z = 0.5 if direction > 0 else -0.5
                print('BLOCKED -> turn %.1f' % (direction*57.3))
            else:
                if random.random() < 0.05:  # 5% chance to change direction
                    direction = random.uniform(-0.5, 0.5)
                msg.linear.x = 0.15
                msg.angular.z = direction
            self.pub.publish(msg)
            self.rate.sleep()
            if time.time() - t0 > 180:  # 3 min limit
                break
        self.pub.publish(Twist())
        print('DONE')

if __name__ == '__main__':
    RandomExplorer().run()
