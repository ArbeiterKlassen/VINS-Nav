#!/usr/bin/env python3
"""Comprehensive exploration: drives turtlebot3 through the house systematically."""
import rospy
from geometry_msgs.msg import Twist
import time

class FullExplorer:
    def __init__(self):
        self.pub = None
        self.rate = None

    def drive(self, lin_x, ang_z, duration):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        end = time.time() + duration
        while time.time() < end and not rospy.is_shutdown():
            self.pub.publish(msg)
            self.rate.sleep()

    def stop(self):
        self.pub.publish(Twist())
        rospy.sleep(0.5)
        self.pub.publish(Twist())

    def run(self):
        rospy.init_node('full_explore')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        rospy.sleep(2)
        rospy.loginfo("Starting comprehensive exploration...")

        # Pattern: systematic sweep through turtlebot3_house
        # The house is roughly 10m x 10m. Robot starts near (-3,1).
        # Move through corridors and rooms in a lawnmower pattern.

        segments = [
            # (linear_x, angular_z, duration, label)
            # Phase 1: exit corner, head into main area
            (0.0, 0.5, 3.0, "turn to face open"),
            (0.2, 0.0, 5.0, "forward into corridor"),

            # Phase 2: sweep left side
            (0.0, 0.5, 1.5, "turn"),
            (0.2, 0.0, 4.0, "forward"),
            (0.0, -0.5, 1.5, "turn"),
            (0.2, 0.0, 3.0, "forward"),

            # Phase 3: cross to right side
            (0.0, -0.5, 3.0, "u-turn"),
            (0.2, 0.0, 6.0, "forward long"),

            # Phase 4: sweep right side rooms
            (0.0, -0.5, 1.5, "turn into room"),
            (0.15, 0.0, 3.0, "enter room"),
            (0.0, 0.5, 3.0, "turn around in room"),
            (0.15, 0.0, 3.0, "exit room"),

            # Phase 5: explore back area
            (0.0, -0.5, 2.0, "turn"),
            (0.2, 0.0, 5.0, "forward back corridor"),
            (0.0, 0.5, 2.0, "turn"),
            (0.2, 0.0, 4.0, "forward"),

            # Phase 6: middle sweep
            (0.0, 0.5, 4.0, "turn around"),
            (0.15, 0.0, 7.0, "long forward sweep"),

            # Phase 7: zigzag through center
            (0.0, -0.5, 1.5, "zig"),
            (0.2, 0.0, 2.5, "zag forward"),
            (0.0, 0.5, 1.5, "zig"),
            (0.2, 0.0, 2.5, "zag forward"),
            (0.0, -0.5, 1.5, "zig"),
            (0.2, 0.0, 2.5, "zag forward"),

            # Phase 8: return sweep
            (0.0, 0.5, 3.0, "turn back"),
            (0.2, 0.0, 6.0, "return sweep"),
        ]

        total_time = sum(d for _, _, d, _ in segments)
        rospy.loginfo("Total estimated time: %.0f seconds", total_time)

        for i, (lin, ang, dur, label) in enumerate(segments):
            rospy.loginfo("[%d/%d] %s (%.1fs)", i+1, len(segments), label, dur)
            self.drive(lin, ang, dur)
            self.stop()
            rospy.sleep(0.3)

        self.stop()
        rospy.loginfo("Exploration complete! Export map with:")
        rospy.loginfo("  rosrun map_server map_saver map:=/rtabmap/grid_map -f ~/catkin_ws_ov/maps/house_full")

if __name__ == '__main__':
    FullExplorer().run()
