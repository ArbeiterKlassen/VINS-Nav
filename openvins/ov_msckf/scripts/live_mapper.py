#!/usr/bin/env python3
"""
live_mapper.py -- online occupancy mapper that mirrors build_map.py.

WHY THIS EXISTS
---------------
RTAB-Map's Grid/FromDepth projection produced maps whose walls are fragmented,
while the offline `build_map.py` (which uses the same depth stream and poses)
produced a map that matches the hand-annotated floor plan much more closely.
The difference is not the data -- it is what is done with it:

  build_map.py                     RTAB-Map grid
  ---------------------------      ---------------------------
  splits points by height into     treats the cloud as one blob,
    wall / floor, keeps walls        so floor points dilute walls
  pure hit accumulation            ray tracing erodes thin walls
  no decimation / noise filter     decimation + filtering
  stride 5 over the depth image    octree/2D projection path

This node does exactly what build_map.py does, but online: it projects each
depth frame into the VIO global frame using the live odometry, classifies
points by height, accumulates hit counts, and publishes the resulting grid on
/map plus a /static_map service -- the two interfaces EGO-Planner needs.

It does NOT do SLAM. It relies on /ov_msckf/odomimu for pose; RTAB-Map can keep
running alongside for map->odom correction.

TOPICS
------
  IN : /camera/depth/image_raw   sensor_msgs/Image        (32FC1)
       /camera/rgb/camera_info   sensor_msgs/CameraInfo
       /ov_msckf/odomimu         nav_msgs/Odometry        (global -> imu)
  OUT: /map                     nav_msgs/OccupancyGrid    (latched, ~1 Hz)
       /static_map              nav_msgs/GetMap service
"""

import collections
import threading

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap, GetMapResponse
from sensor_msgs.msg import CameraInfo, Image


class LiveMapper:
    def __init__(self):
        rospy.init_node("live_mapper")

        # ---- geometry / classification (same defaults as build_map.py) ----
        self.res = rospy.get_param("~resolution", 0.05)
        self.stride = rospy.get_param("~stride", 5)
        # Process 1 frame in `frame_stride`. build_map.py does the same, and it
        # matters: at 16 Hz every frame carries a slightly different pose error,
        # so accumulating all of them smears each wall across many cells and the
        # map degrades into a blob. Subsampling keeps the walls crisp.
        self.frame_stride = rospy.get_param("~frame_stride", 5)
        self._frame_count = 0
        self.wall_zmin = rospy.get_param("~wall_zmin", 0.15)
        self.wall_zmax = rospy.get_param("~wall_zmax", 3.0)
        self.floor_zmin = rospy.get_param("~floor_zmin", -0.5)
        self.floor_zmax = rospy.get_param("~floor_zmax", 0.15)
        self.range_max = rospy.get_param("~range_max", 6.0)
        # occupancy rule: a cell is a wall if wall hits dominate floor hits
        self.wall_ratio = rospy.get_param("~wall_ratio", 5.0)
        # free rule: enough floor evidence and no wall evidence
        self.free_min_floor = rospy.get_param("~free_min_floor", 20)
        # grid extent, centred on the VIO origin (metres, each side)
        self.extent = rospy.get_param("~extent", 15.0)
        self.publish_period = rospy.get_param("~publish_period", 1.0)

        self.n = int(2 * self.extent / self.res)
        self.gw = np.zeros((self.n, self.n), dtype=np.int32)   # wall hits
        self.gf = np.zeros((self.n, self.n), dtype=np.int32)   # floor hits
        self.lock = threading.Lock()

        self.fx = self.fy = self.cx = self.cy = None
        self.pose_buf = collections.deque(maxlen=4000)   # (t, x,y,z, qx,qy,qz,qw)
        self.frames = 0
        self.dropped = 0

        # Pose source: "odom" (live VIO) or "tf" (look up global->imu in TF).
        # "tf" lets the mapper run on a recorded trajectory -- e.g. replaying the
        # bag's own /tf -- which isolates mapper behaviour from VIO behaviour.
        self.pose_source = rospy.get_param("~pose_source", "odom")
        self.tf_buffer = None
        if self.pose_source == "tf":
            import tf2_ros
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.cinfo_cb, queue_size=1)
        if self.pose_source == "odom":
            rospy.Subscriber("/ov_msckf/odomimu", Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_cb, queue_size=1)

        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.srv = rospy.Service("/static_map", GetMap, self.handle_static_map)
        rospy.Timer(rospy.Duration(self.publish_period), self.publish_cb)

        # optional diagnostic: dump the poses actually used for projection
        self.pose_log = None
        log_path = rospy.get_param("~pose_log", "")
        if log_path:
            self.pose_log = open(log_path, "w")
            self.pose_log.write("t,x,y,z,qx,qy,qz,qw\n")
            rospy.loginfo("live_mapper: logging used poses to %s", log_path)

        rospy.loginfo("live_mapper: %dx%d grid @ %.3fm, extent +-%.1fm, stride %d",
                      self.n, self.n, self.res, self.extent, self.stride)

    # ------------------------------------------------------------------
    def cinfo_cb(self, msg):
        if self.fx is None:
            K = np.array(msg.K).reshape(3, 3)
            self.fx, self.fy = K[0, 0], K[1, 1]
            self.cx, self.cy = K[0, 2], K[1, 2]
            rospy.loginfo("live_mapper: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                          self.fx, self.fy, self.cx, self.cy)

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose_buf.append((msg.header.stamp.to_sec(),
                              p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def lookup_pose(self, stamp):
        """Nearest pose to `stamp` (within 0.1 s), or None."""
        if self.pose_source == "tf":
            if self.tf_buffer is None:
                return None
            try:
                tr = self.tf_buffer.lookup_transform(
                    "global", "imu", rospy.Time.from_sec(stamp),
                    rospy.Duration(0.1))
            except Exception:
                return None
            t, r = tr.transform.translation, tr.transform.rotation
            return (stamp, t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        if not self.pose_buf:
            return None
        best = min(self.pose_buf, key=lambda r: abs(r[0] - stamp))
        if abs(best[0] - stamp) > 0.1:
            return None
        return best

    # ------------------------------------------------------------------
    def depth_cb(self, msg):
        if self.fx is None:
            return
        self._frame_count += 1
        if self._frame_count % self.frame_stride != 0:
            return
        pose = self.lookup_pose(msg.header.stamp.to_sec())
        if pose is None:
            self.dropped += 1
            return
        _, ox, oy, oz, qx, qy, qz, qw = pose
        if self.pose_log is not None:
            self.pose_log.write("%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n"
                                % (msg.header.stamp.to_sec(), ox, oy, oz, qx, qy, qz, qw))
            self.pose_log.flush()

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        h, w = depth.shape
        v, u = np.mgrid[0:h:self.stride, 0:w:self.stride]
        Z = depth[v, u]
        valid = np.isfinite(Z) & (Z > 0.01) & (Z < self.range_max)
        if not valid.any():
            return

        Uv, Vv, Zv = u[valid].astype(np.float32), v[valid].astype(np.float32), Z[valid]
        Xc = (Uv - self.cx) * Zv / self.fx
        Yc = (Vv - self.cy) * Zv / self.fy

        # camera optical -> IMU (verified in build_map.py against the depth Z range)
        Xi, Yi, Zi = Zv, -Xc, -Yc

        # quaternion (IMU -> global)
        R00 = qw * qw + qx * qx - qy * qy - qz * qz
        R01 = 2 * (qx * qy - qw * qz)
        R02 = 2 * (qx * qz + qw * qy)
        R10 = 2 * (qx * qy + qw * qz)
        R11 = qw * qw - qx * qx + qy * qy - qz * qz
        R12 = 2 * (qy * qz - qw * qx)
        R20 = 2 * (qx * qz - qw * qy)
        R21 = 2 * (qy * qz + qw * qx)
        R22 = qw * qw - qx * qx - qy * qy + qz * qz

        Xw = R00 * Xi + R01 * Yi + R02 * Zi + ox
        Yw = R10 * Xi + R11 * Yi + R12 * Zi + oy
        Zw = R20 * Xi + R21 * Yi + R22 * Zi + oz

        wall = (Zw > self.wall_zmin) & (Zw < self.wall_zmax)
        floor = (Zw > self.floor_zmin) & (Zw < self.floor_zmax)

        with self.lock:
            for mask, grid in ((wall, self.gw), (floor, self.gf)):
                if not mask.any():
                    continue
                gx = ((Xw[mask] + self.extent) / self.res).astype(np.int32)
                gy = ((Yw[mask] + self.extent) / self.res).astype(np.int32)
                keep = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
                np.add.at(grid, (gy[keep], gx[keep]), 1)

        self.frames += 1
        if self.frames % 200 == 0:
            with self.lock:
                occ = int(((self.gw > 0) & (self.gw * self.wall_ratio > self.gf)).sum())
                fre = int(((self.gf > self.free_min_floor) & (self.gw == 0)).sum())
            rospy.loginfo("live_mapper: %d frames, wall pts %d/%d (%.0f%%), "
                          "Z[%.2f..%.2f] p50=%.2f, cells occ=%d free=%d",
                          self.frames, int(wall.sum()), int(wall.size),
                          100.0 * wall.sum() / max(1, wall.size),
                          float(Zw.min()), float(Zw.max()), float(np.median(Zw)),
                          occ, fre)

    # ------------------------------------------------------------------
    def build_grid_msg(self):
        with self.lock:
            gw, gf = self.gw, self.gf
            occ = (gw > 0) & (gw * self.wall_ratio > gf)
            free = (gf > self.free_min_floor) & (gw == 0)
            # OccupancyGrid convention: -1 = unknown, 0 = free, 100 = occupied.
            # (205 is the *PGM* unknown value -- using it here makes map_saver
            # render every unknown cell as occupied.)
            pgm = np.full((self.n, self.n), -1, dtype=np.int8)
            pgm[free] = 0
            pgm[occ] = 100
        # OccupancyGrid row 0 is the lowest y, same as our gy axis -> no flip
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.resolution = self.res
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = -self.extent
        msg.info.origin.position.y = -self.extent
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = pgm.flatten().tolist()
        return msg

    def publish_cb(self, _evt):
        if self.frames == 0:
            return
        self.map_pub.publish(self.build_grid_msg())

    def handle_static_map(self, _req):
        if self.frames == 0:
            return GetMapResponse()
        return GetMapResponse(self.build_grid_msg())


if __name__ == "__main__":
    try:
        LiveMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
