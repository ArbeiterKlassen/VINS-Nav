#!/usr/bin/env python3
"""
vio_odom_bridge.py — Bridge OpenVINS VIO to ego-planner odometry + TF.

====================================
WHY THIS NODE EXISTS
====================================
OpenVINS and ego-planner use different frame conventions:

  OpenVINS /ov_msckf/odomimu:    frame_id="global"  child_frame_id="imu"
  ego-planner /odom_world:       frame_id="odom"    child_frame_id="base_footprint"

This node:
  1. Subscribes to /ov_msckf/odomimu
  2. Transforms imu pose -> base_footprint pose (Z offset -0.078 m)
  3. Publishes /odom_world (nav_msgs/Odometry) at 50 Hz
  4. Publishes odom->base_footprint TF at 50 Hz (DYNAMIC, from VIO)

====================================
TF TREE (ego-planner integrated mode)
====================================
  map ------> global ------> odom -------> base_footprint --> base_link
    (static      (static       (DYNAMIC         (URDF
    identity)    identity)     from VIO,        robot_state_publisher)
                               THIS NODE)

  The odom frame is FIXED (world-fixed, same as map).
  The base_footprint frame MOVES as the robot moves (VIO odometry).
  This matches ego-planner's semantic model exactly.

  Compare with the ORIGINAL ego-planner TF tree:
    map ---> world ---> odom ---> base_link
      (static)   (static)  (DYNAMIC from fake_odom)

  We replace fake_odom's cmd_vel integration with real VIO pose.

====================================
TOPICS
====================================
  INPUT:  /ov_msckf/odomimu  (nav_msgs/Odometry,   global->imu)
  OUTPUT: /odom_world         (nav_msgs/Odometry,   odom->base_footprint)
  OUTPUT: TF odom->base_footprint (geometry_msgs/TransformStamped)
"""

import rospy
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class VIOOdomBridge:
    def __init__(self):
        rospy.init_node("vio_odom_bridge")

        # ---- parameters ----
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.publish_rate = rospy.get_param("~publish_rate", 50.0)
        # Z-offset from IMU to base_footprint (negative = base_footprint below IMU)
        self.imu_to_base_z = rospy.get_param("~imu_to_base_z", -0.078)

        # ---- publisher ----
        self.odom_pub = rospy.Publisher("/odom_world", Odometry, queue_size=10)
        self.tf_pub = rospy.Publisher("/tf", rospy.AnyMsg, queue_size=100)

        # ---- subscriber ----
        self.latest_vio = None
        rospy.Subscriber("/ov_msckf/odomimu", Odometry, self.vio_cb, queue_size=10)

        # ---- timer for steady output ----
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.timer_cb
        )

        rospy.loginfo(
            "VIO Odom Bridge ready: /ov_msckf/odomimu -> /odom_world + TF  (%s -> %s) @ %.0f Hz",
            self.odom_frame, self.robot_frame, self.publish_rate,
        )

    # ------------------------------------------------------------------
    def vio_cb(self, msg):
        """Cache latest VIO odometry."""
        self.latest_vio = msg

    # ------------------------------------------------------------------
    def transform_imu_to_base(self, p_imu, q_imu):
        """Apply fixed Z offset from IMU to base_footprint.

        IMU is mounted 0.078 m above base_footprint (from URDF).
        In the IMU frame (Z up), base_footprint is at (0, 0, -0.078).
        Rotate this offset by the IMU orientation to express it in world frame.
        """
        R = tft.quaternion_matrix(q_imu)[:3, :3]
        offset = np.array([0.0, 0.0, self.imu_to_base_z])
        p_base = p_imu + R @ offset
        return p_base, q_imu  # orientation unchanged

    # ------------------------------------------------------------------
    def timer_cb(self, event):
        if self.latest_vio is None:
            return

        msg = self.latest_vio
        now = rospy.Time.now()

        # -- VIO pose (global -> imu) --
        p_imu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        q_imu = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])

        # -- Transform to base_footprint --
        p_base, q_base = self.transform_imu_to_base(p_imu, q_imu)

        # === Publish /odom_world (odom -> base_footprint) ===
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_frame
        odom.pose.pose.position.x = p_base[0]
        odom.pose.pose.position.y = p_base[1]
        odom.pose.pose.position.z = p_base[2]
        odom.pose.pose.orientation.x = q_base[0]
        odom.pose.pose.orientation.y = q_base[1]
        odom.pose.pose.orientation.z = q_base[2]
        odom.pose.pose.orientation.w = q_base[3]

        # Velocity
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.linear.z = msg.twist.twist.linear.z
        odom.twist.twist.angular.x = msg.twist.twist.angular.x
        odom.twist.twist.angular.y = msg.twist.twist.angular.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z

        # Covariance
        odom.pose.covariance = list(msg.pose.covariance)
        odom.twist.covariance = list(msg.twist.covariance)

        self.odom_pub.publish(odom)

        # === Publish TF  odom -> base_footprint ===
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.robot_frame
        tf_msg.transform.translation.x = p_base[0]
        tf_msg.transform.translation.y = p_base[1]
        tf_msg.transform.translation.z = p_base[2]
        tf_msg.transform.rotation.x = q_base[0]
        tf_msg.transform.rotation.y = q_base[1]
        tf_msg.transform.rotation.z = q_base[2]
        tf_msg.transform.rotation.w = q_base[3]
        self.tf_pub.publish(tf_msg)


# ======================================================================
if __name__ == "__main__":
    try:
        VIOOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
