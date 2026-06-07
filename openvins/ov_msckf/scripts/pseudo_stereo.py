#!/usr/bin/env python3
"""
Pseudo-Stereo Node: generates a virtual right camera image from RGB + depth.
Uses the stereo disparity formula: d = fx * baseline / Z.

Left camera:  /camera/rgb/image_raw (source)
Right camera: /camera/right/image_raw (generated, published here)
"""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class PseudoStereo:
    def __init__(self):
        # Baseline in meters (similar to RealSense D435)
        self.baseline = rospy.get_param('~baseline', 0.08)

        self.bridge = CvBridge()
        self.K = None       # 3x3 intrinsics from left camera_info
        self.fx = None
        self.latest_depth = None
        self.latest_rgb = None
        self.rgb_stamp = None

        # Subscribers
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_cb, queue_size=1)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.cinfo_cb)

        # Publishers
        self.right_img_pub = rospy.Publisher('/camera/right/image_raw', Image, queue_size=1)
        self.right_cinfo_pub = rospy.Publisher('/camera/right/camera_info', CameraInfo, queue_size=1)

        # Process at camera rate
        self.timer = rospy.Timer(rospy.Duration(0.05), self.process)

        rospy.loginfo("PseudoStereo started, baseline=%.3fm", self.baseline)

    def cinfo_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3)
            self.fx = self.K[0, 0]
            rospy.loginfo("Camera intrinsics: fx=%.2f, cx=%.2f, cy=%.2f",
                         self.K[0,0], self.K[0,2], self.K[1,2])

    def rgb_cb(self, msg):
        self.latest_rgb = msg
        self.rgb_stamp = msg.header.stamp

    def depth_cb(self, msg):
        self.latest_depth = msg

    def process(self, event):
        if self.latest_rgb is None or self.latest_depth is None or self.fx is None:
            return

        stamp = self.latest_rgb.header.stamp
        if abs((stamp - self.latest_depth.header.stamp).to_sec()) > 0.1:
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(self.latest_depth, '32FC1')
        except Exception as e:
            rospy.logwarn_throttle(5, "Conversion error: %s" % str(e))
            return

        h, w = rgb.shape[:2]
        right = np.zeros_like(rgb)

        # Compute disparity: d = fx * baseline / Z
        valid = np.isfinite(depth) & (depth > 0.01)
        disparity = np.zeros_like(depth, dtype=np.float32)
        disparity[valid] = (self.fx * self.baseline) / depth[valid]

        # Per-row forward warp with vectorized Z-buffer via np.unique
        filled = np.zeros((h, w), dtype=bool)
        u_cols = np.arange(w, dtype=np.float32)

        for v in range(h):
            row_valid = valid[v]
            if not row_valid.any():
                continue
            # Source positions and target positions for this row
            u_src = np.where(row_valid)[0]
            disp = disparity[v, u_src]
            u_dst = np.clip(np.round(u_src - disp).astype(np.int32), 0, w - 1)
            # Sort by disparity descending (closest first)
            order = np.argsort(-disp)
            u_src_s = u_src[order]
            u_dst_s = u_dst[order]
            # np.unique returns first occurrence of each dest = closest = Z-buffer
            _, first = np.unique(u_dst_s, return_index=True)
            right[v, u_dst_s[first]] = rgb[v, u_src_s[first]]
            filled[v, u_dst_s[first]] = True

        # Inpaint remaining holes with Navier-Stokes (smoother than TELEA)
        if not filled.all():
            right = cv2.inpaint(right, (~filled).astype(np.uint8), 3, cv2.INPAINT_NS)

        # Publish right image
        right_msg = self.bridge.cv2_to_imgmsg(right, 'bgr8')
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = self.latest_rgb.header.frame_id
        self.right_img_pub.publish(right_msg)

        # Publish right camera info (same as left, but adjusted P matrix)
        if self.latest_rgb is not None:
            cinfo = CameraInfo()
            cinfo.header.stamp = stamp
            cinfo.header.frame_id = self.latest_rgb.header.frame_id
            cinfo.height = h
            cinfo.width = w
            cinfo.distortion_model = 'plumb_bob'
            cinfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            cinfo.K = [self.fx, 0, self.K[0,2],
                       0, self.fx, self.K[1,2],
                       0, 0, 1]
            # P matrix: K * [I | -baseline/fx] — right camera shifted by baseline
            cinfo.P = [self.fx, 0, self.K[0,2], -self.fx * self.baseline,
                       0, self.fx, self.K[1,2], 0,
                       0, 0, 1, 0]
            cinfo.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            self.right_cinfo_pub.publish(cinfo)

if __name__ == '__main__':
    rospy.init_node('pseudo_stereo')
    PseudoStereo()
    rospy.spin()
