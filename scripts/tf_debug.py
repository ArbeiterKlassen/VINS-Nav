#!/usr/bin/env python3
"""Debug TF2 buffer to verify transform tree."""
import rospy
import tf2_ros

rospy.init_node('tf_debug')
buf = tf2_ros.Buffer(rospy.Duration(30))
tf2_ros.TransformListener(buf)
rospy.sleep(2.0)
now = rospy.Time.now()

print("Time now: %.3f" % now.to_sec())

pairs = [('map','odom'), ('odom','map'), ('map','base_footprint'), ('odom','base_footprint'),
         ('map','base_link'), ('map','base_scan')]
for target, source in pairs:
    try:
        tf = buf.lookup_transform(target, source, now, rospy.Duration(2.0))
        print("OK: %-20s -> %-20s = (%.3f, %.3f, %.3f)" % (
            target, source,
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z))
    except Exception as e:
        print("FAIL: %-20s -> %-20s : %s" % (target, source, str(e)[:90]))

print()
print("=== all_frames_as_string ===")
print(buf.all_frames_as_string())
