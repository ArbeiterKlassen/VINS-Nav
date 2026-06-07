#!/usr/bin/env python3
"""Publish map->odom TF at 10Hz from AMCL pose for navigation."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft

class AMCLTFPublisher:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.latest_pose = None    # robot pose in map frame
        self.latest_odom = None    # robot pose in odom frame
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.rate = rospy.Rate(10)
        rospy.loginfo("AMCL TF Publisher started (10Hz)")

    def pose_cb(self, msg):
        self.latest_pose = msg.pose.pose

    def odom_cb(self, msg):
        self.latest_odom = msg.pose.pose

    def run(self):
        while not rospy.is_shutdown():
            if self.latest_pose and self.latest_odom:
                # map->odom = map->base * base->odom
                # map->base from AMCL pose, base->odom from odometry
                # map->odom = map->base * (odom->base)^-1
                mp = self.latest_pose.position
                mq = self.latest_pose.orientation
                op = self.latest_odom.position
                oq = self.latest_odom.orientation

                # robot pose in map: T_m_b
                T_m_b = tft.translation_matrix([mp.x, mp.y, mp.z])
                T_m_b[:3,:3] = tft.quaternion_matrix([mq.x, mq.y, mq.z, mq.w])[:3,:3]

                # robot pose in odom: T_o_b
                T_o_b = tft.translation_matrix([op.x, op.y, op.z])
                T_o_b[:3,:3] = tft.quaternion_matrix([oq.x, oq.y, oq.z, oq.w])[:3,:3]

                # map->odom: T_m_o = T_m_b * inv(T_o_b)
                T_m_o = T_m_b.dot(tft.inverse_matrix(T_o_b))
                p = tft.translation_from_matrix(T_m_o)
                q = tft.quaternion_from_matrix(T_m_o)

                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "odom"
                t.transform.translation.x = p[0]
                t.transform.translation.y = p[1]
                t.transform.translation.z = p[2]
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                self.br.sendTransform(t)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('amcl_tf_pub')
    AMCLTFPublisher().run()
