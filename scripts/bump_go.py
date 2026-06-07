#!/usr/bin/env python3
"""Bump-and-go explorer: drive forward, reverse+turn on obstacle."""
import rospy, time, random
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class BumpGo:
    def __init__(self):
        self.scan = None
        rospy.init_node('bump_go')
        rospy.Subscriber('/scan', LaserScan, self.cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def cb(self, m):
        self.scan = np.array(m.ranges)

    def blocked(self):
        if self.scan is None:
            return True
        r = self.scan.copy()
        r[np.isinf(r)] = 10.0
        n = len(r)
        front = r[n//3:2*n//3]  # center 60 degrees
        return np.min(front) < 0.4

    def drive(self, lin, ang, dur):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < dur and not rospy.is_shutdown():
            self.pub.publish(msg)
            self.rate.sleep()

    def stop(self):
        self.pub.publish(Twist())
        time.sleep(0.3)
        self.pub.publish(Twist())

    def run(self):
        rospy.sleep(3)
        print("Bump-Go Explorer started")
        t_start = time.time()
        while not rospy.is_shutdown() and time.time() - t_start < 300:  # 5 min
            if self.blocked():
                print("BUMP! reversing...")
                self.drive(-0.1, 0, 1.5)       # reverse 1.5s
                self.stop()
                turn_dir = random.choice([-1, 1])
                turn_time = random.uniform(1.5, 3.0)
                print("TURN %.1fs dir=%d" % (turn_time, turn_dir))
                self.drive(0, 0.5 * turn_dir, turn_time)
                self.stop()
            else:
                self.drive(0.15, 0, 0.5)  # forward 0.5s chunks
        self.stop()
        print("DONE")

if __name__ == '__main__':
    BumpGo().run()
