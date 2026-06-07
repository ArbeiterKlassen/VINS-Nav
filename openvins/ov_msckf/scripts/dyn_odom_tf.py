#!/usr/bin/env python3
"""Publish dynamic global->odom TF from VIO odometry."""
import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def cb(msg, br):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = 'global'
    t.child_frame_id = 'odom'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransform(t)

def main():
    rospy.init_node('dyn_odom_tf')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/ov_msckf/odomimu', Odometry, cb, br)
    rospy.spin()

if __name__ == '__main__':
    main()
