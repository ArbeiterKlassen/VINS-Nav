#!/usr/bin/env python3
"""Bridge: publish global->imu TF from /ov_msckf/odomimu for RTAB-Map."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomTFPublisher:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.latest = None
        rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.callback)
        self.rate = rospy.Rate(30)  # 30Hz for reliable TF lookups
        rospy.loginfo("Odom TF Publisher started (10Hz)")

    def callback(self, msg):
        t = TransformStamped()
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.latest = t

    def run(self):
        while not rospy.is_shutdown():
            if self.latest:
                self.latest.header.stamp = rospy.Time.now()
                self.br.sendTransform(self.latest)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odom_tf_pub')
    OdomTFPublisher().run()
