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

        # Hole-filling strategy after warping. The right image has ~40% holes at
        # typical indoor depth (disocclusion + depth dropouts). Measured cost per
        # 640x480 frame on this machine:
        #   inpaint_ns  ~170 ms   (original) -> caps the node at ~5 Hz
        #   inpaint_ds  ~ 17 ms   (TELEA at 1/4 scale, upsampled)
        #   morph       ~  8 ms   (morphological close)
        #   none        ~  0 ms
        # VIO needs ~28 Hz stereo pairs, so the NS default starves it and the
        # estimator drifts. Set ~fill_mode:=inpaint_ns to reproduce old behavior.
        self.fill_mode = rospy.get_param('~fill_mode', 'morph')
        self.fill_scale = int(rospy.get_param('~fill_scale', 4))

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

        rospy.loginfo("PseudoStereo started, baseline=%.3fm, fill_mode=%s",
                      self.baseline, self.fill_mode)

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

        # Per-row forward warp with a Z-buffer.
        # lexsort puts destination columns in ascending order with the closest
        # (largest-disparity) source first within each column; a boolean diff then
        # picks the first entry of each group. Same output as np.unique-based
        # grouping, ~25% faster (np.unique re-sorts the whole row).
        filled = np.zeros((h, w), dtype=bool)

        for v in range(h):
            row_valid = valid[v]
            if not row_valid.any():
                continue
            u_src = np.where(row_valid)[0]
            disp = disparity[v, u_src]
            u_dst = np.clip(np.round(u_src - disp).astype(np.int32), 0, w - 1)

            order = np.lexsort((-disp, u_dst))
            u_src_s = u_src[order]
            u_dst_s = u_dst[order]

            starts = np.empty(len(u_dst_s), dtype=bool)
            starts[0] = True
            starts[1:] = u_dst_s[1:] != u_dst_s[:-1]

            u_src_s = u_src_s[starts]
            u_dst_s = u_dst_s[starts]
            right[v, u_dst_s] = rgb[v, u_src_s]
            filled[v, u_dst_s] = True

        if not filled.all():
            right = self.fill_holes(right, filled)

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

    def fill_holes(self, right, filled):
        """Fill holes left by the forward warp, per ~fill_mode."""
        if self.fill_mode == 'none':
            return right

        if self.fill_mode == 'morph':
            # Dilate real pixels into the holes (~8 ms/frame).
            kernel = np.ones((5, 5), np.uint8)
            closed = cv2.morphologyEx(right, cv2.MORPH_CLOSE, kernel)
            out = right.copy()
            out[~filled] = closed[~filled]
            return out

        mask = (~filled).astype(np.uint8)

        if self.fill_mode == 'inpaint_ds':
            # Inpaint at 1/fill_scale resolution, then upsample the holes only.
            s = max(1, self.fill_scale)
            sh, sw = max(1, right.shape[0] // s), max(1, right.shape[1] // s)
            small_img = cv2.resize(right, (sw, sh), interpolation=cv2.INTER_NEAREST)
            small_mask = cv2.resize(mask, (sw, sh), interpolation=cv2.INTER_NEAREST)
            small_img = cv2.inpaint(small_img, small_mask, 3, cv2.INPAINT_TELEA)
            up = cv2.resize(small_img, (right.shape[1], right.shape[0]),
                            interpolation=cv2.INTER_LINEAR)
            out = right.copy()
            out[~filled] = up[~filled]
            return out

        if self.fill_mode == 'inpaint_ns':
            # Original behavior: full-resolution Navier-Stokes (~170 ms/frame).
            return cv2.inpaint(right, mask, 3, cv2.INPAINT_NS)

        rospy.logwarn_throttle(10, "pseudo_stereo: unknown fill_mode '%s', leaving holes",
                               self.fill_mode)
        return right


if __name__ == '__main__':
    rospy.init_node('pseudo_stereo')
    PseudoStereo()
    rospy.spin()
