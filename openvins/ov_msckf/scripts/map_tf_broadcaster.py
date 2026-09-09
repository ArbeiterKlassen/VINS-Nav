#!/usr/bin/env python3
"""Broadcast RTAB-Map's map->odom TF for navigation (10Hz)."""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

# Dynamically load rtabmap_ros MapData type
try:
    from rtabmap_ros.msg import MapData
except ImportError:
    import roslib.message
    MapData = roslib.message.get_message_class('rtabmap_ros/MapData')

class MapTFBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.latest = None
        if MapData:
            rospy.Subscriber('/rtabmap/mapData', MapData, self.callback)
        else:
            rospy.Subscriber('/rtabmap/mapData', rospy.AnyMsg, self.callback_any)
        self.rate = rospy.Rate(10)
        rospy.loginfo("Map TF Broadcaster started (10Hz)")

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation = msg.graph.mapToOdom.translation
        t.transform.rotation = msg.graph.mapToOdom.rotation
        self.latest = t

    def callback_any(self, msg):
        # Minimal binary parse for geometry_msgs/Transform in mapData.graph.mapToOdom
        try:
            import struct
            buf = bytes(msg._buff)
            # Skip: header(4+8+4+frame_id_len) + graph_header(4+8+4+frame_id_len)
            pos = 16
            flen = struct.unpack_from('<I', buf, pos)[0]; pos += 4 + flen
            pos += 16
            flen = struct.unpack_from('<I', buf, pos)[0]; pos += 4 + flen
            tx, ty, tz = struct.unpack_from('<ddd', buf, pos); pos += 24
            rx, ry, rz, rw = struct.unpack_from('<dddd', buf, pos)
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x = rx
            t.transform.rotation.y = ry
            t.transform.rotation.z = rz
            t.transform.rotation.w = rw
            self.latest = t
        except Exception as e:
            rospy.logwarn_throttle(5, "Parse error: %s" % str(e))

    def run(self):
        while not rospy.is_shutdown():
            if self.latest:
                self.latest.header.stamp = rospy.Time.now()
                self.br.sendTransform(self.latest)
            try:
                self.rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                # Happens when a bag replay starts/ends and /clock jumps.
                # Rebuild the rate against the new clock instead of dying.
                self.rate = rospy.Rate(10)
            except rospy.ROSInterruptException:
                break

if __name__ == '__main__':
    rospy.init_node('map_tf_broadcaster')
    MapTFBroadcaster().run()
