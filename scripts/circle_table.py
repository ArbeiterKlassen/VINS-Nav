#!/usr/bin/env python3
"""Circle around the table near spawn point (-3, 1) in turtlebot3 house."""
import rospy, time, math
from geometry_msgs.msg import Twist

class CircleTable:
    def __init__(self):
        rospy.init_node('circle_table')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        time.sleep(2)  # let publishers connect

    def drive(self, lin, ang, duration, label=""):
        msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < duration and not rospy.is_shutdown():
            self.pub.publish(msg)
            time.sleep(0.05)
        self.stop()
        if label:
            rospy.loginfo("[%s] %.1fs lin=%.2f ang=%.2f", label, duration, lin, ang)
        time.sleep(0.3)

    def stop(self):
        self.pub.publish(Twist())
        time.sleep(0.2)
        self.pub.publish(Twist())

    def run(self):
        rospy.loginfo("=== Circle Table: %d loops ===", rospy.get_param('~loops', 3))
        loops = rospy.get_param('~loops', 3)

        # Pattern: go around the table area
        # Table is roughly 1.5m x 1m near spawn (-3, 1)
        # Drive a rectangular loop: forward -> turn -> forward -> turn -> ...
        for loop in range(loops):
            rospy.loginfo("Loop %d/%d", loop+1, loops)
            # Reduce speed slightly each loop for map refinement
            speed = max(0.08, 0.2 - loop * 0.04)
            turn_speed = max(0.3, 0.5 - loop * 0.07)

            self.drive(speed, 0, 4.0, "fwd1")
            self.drive(0, turn_speed, 1.6, "turn1")
            self.drive(speed, 0, 3.0, "fwd2")
            self.drive(0, turn_speed, 1.6, "turn2")
            self.drive(speed, 0, 4.0, "fwd3")
            self.drive(0, turn_speed, 1.6, "turn3")
            self.drive(speed, 0, 3.0, "fwd4")
            self.drive(0, turn_speed, 1.6, "turn4")

        self.stop()
        rospy.loginfo("Done. Map ready for export.")

if __name__ == '__main__':
    CircleTable().run()
