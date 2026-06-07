#!/usr/bin/env python3
"""Build 2D occupancy grid directly from bag: TF pose + depth + camera info."""
import rosbag, rospy, sys, os, cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

def build_map(bag_path, output, stride=5):
    bag = rosbag.Bag(bag_path)

    # Pass 1: collect poses from TF (global->odom)
    print("Pass 1: TF poses...")
    odom_poses = {}
    for _, msg, _ in bag.read_messages(topics=['/tf']):
        for trans in msg.transforms:
            if trans.header.frame_id == 'global' and trans.child_frame_id == 'imu':
                odom_poses[trans.header.stamp.to_sec()] = (
                    trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                    trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
                )
    odom_times = np.array(sorted(odom_poses.keys()))
    print(f"  {len(odom_times)} poses")

    # Pass 2: get camera intrinsics
    print("Pass 2: camera intrinsics...")
    fx = fy = cx = cy = None
    for _, msg, _ in bag.read_messages(topics=['/camera/rgb/camera_info']):
        K = np.array(msg.K).reshape(3,3)
        fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
        print(f"  fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
        break

    # Pass 3: process depth frames
    print("Pass 3: building cloud...")
    cloud = []
    count = 0
    # Time range + pose correction for late-trajectory drift
    max_time = float(sys.argv[3]) if len(sys.argv) > 3 else -1
    min_time = float(sys.argv[4]) if len(sys.argv) > 4 else -1
    corr_dx = float(sys.argv[5]) if len(sys.argv) > 5 else 0
    corr_dy = float(sys.argv[6]) if len(sys.argv) > 6 else 0
    for _, msg, _ in bag.read_messages(topics=['/camera/depth/image_raw']):
        stamp = msg.header.stamp.to_sec()
        if max_time > 0 and stamp > max_time: continue
        if min_time > 0 and stamp < min_time: continue
        idx = np.searchsorted(odom_times, stamp)
        if idx >= len(odom_times): idx = len(odom_times)-1
        if idx > 0 and abs(odom_times[idx-1]-stamp) < abs(odom_times[idx]-stamp): idx -= 1
        if abs(odom_times[idx]-stamp) > 0.1: continue

        ox, oy, oz, qx, qy, qz, qw = odom_poses[odom_times[idx]]

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        count += 1
        if count % stride != 0: continue

        h, w = depth.shape
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        Z = depth[v, u]
        valid = np.isfinite(Z) & (Z > 0.01) & (Z < 30)
        if not valid.any(): continue

        Uv, Vv, Zv = u[valid], v[valid], Z[valid]
        Xc = (Uv - cx) * Zv / fx
        Yc = (Vv - cy) * Zv / fy

        # Camera optical -> IMU
        Xi = Zv
        Yi = -Xc
        Zi = -Yc

        # Quaternion rotation matrix (for IMU→global)
        R00 = qw*qw + qx*qx - qy*qy - qz*qz
        R01 = 2*(qx*qy - qw*qz); R02 = 2*(qx*qz + qw*qy)
        R10 = 2*(qx*qy + qw*qz); R11 = qw*qw - qx*qx + qy*qy - qz*qz
        R12 = 2*(qy*qz - qw*qx)
        R20 = 2*(qx*qz - qw*qy); R21 = 2*(qy*qz + qw*qx)
        R22 = qw*qw - qx*qx - qy*qy + qz*qz

        Xw = R00*Xi + R01*Yi + R02*Zi + ox
        Yw = R10*Xi + R11*Yi + R12*Zi + oy
        Zw = R20*Xi + R21*Yi + R22*Zi + oz
        # Apply drift correction: LEFT half (visited later, VIO drift)
        if corr_dx != 0 or corr_dy != 0:
            if ox > -1.0:
                Xw += corr_dx
                Yw += corr_dy
        cloud.append(np.column_stack([Xw, Yw, Zw]))
        if count % 100 == 0: print(f"  {count} frames, {sum(len(c) for c in cloud)} pts")

    bag.close()
    if not cloud: print("ERROR: no points"); return

    pts = np.vstack(cloud)
    pts = pts[np.abs(pts[:,0])<30]; pts = pts[np.abs(pts[:,1])<30]; pts = pts[np.abs(pts[:,2])<10]
    print(f"Total: {len(pts)} pts, X:[{pts[:,0].min():.1f},{pts[:,0].max():.1f}], Y:[{pts[:,1].min():.1f},{pts[:,1].max():.1f}], Z:[{pts[:,2].min():.2f},{pts[:,2].max():.2f}]")

    # 2D grid
    res = 0.05
    xmin, xmax = pts[:,0].min()-2, pts[:,0].max()+2
    ymin, ymax = pts[:,1].min()-2, pts[:,1].max()+2
    w, h = int((xmax-xmin)/res)+1, int((ymax-ymin)/res)+1
    wall = (pts[:,2] > 0.15) & (pts[:,2] < 3.0)
    floor = (pts[:,2] > -0.5) & (pts[:,2] < 0.15)

    gw = np.zeros((h,w), dtype=np.int32); gf = np.zeros((h,w), dtype=np.int32)
    for mask, grid in [(wall, gw), (floor, gf)]:
        if mask.any():
            gx = ((pts[mask,0]-xmin)/res).astype(np.int32)
            gy = ((pts[mask,1]-ymin)/res).astype(np.int32)
            v = (gx>=0)&(gx<w)&(gy>=0)&(gy<h)
            np.add.at(grid, (gy[v], gx[v]), 1)

    # Occupied: wall points dominant over floor
    occ = np.zeros((h,w), dtype=np.int8)
    occ[(gw > 0) & (gw * 5 > gf)] = 100   # wall pts dominant over floor
    # Free: well-observed, no walls
    free = (gf > 20) & (gw == 0)
    # Unknown: sparse, or mixed wall/floor
    pgm = np.full((h,w), 205, dtype=np.uint8)
    pgm[occ > 0] = 0
    pgm[free] = 254
    cv2.imwrite(output+'.pgm', pgm)
    with open(output+'.yaml','w') as f:
        f.write(f"image: {os.path.abspath(output+'.pgm')}\nresolution: {res}\norigin: [{xmin}, {ymin}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"Saved: {output}.pgm ({w}x{h})")

if __name__ == '__main__':
    build_map(sys.argv[1] if len(sys.argv)>1 else os.path.expanduser('~/house_full.bak'),
              sys.argv[2] if len(sys.argv)>2 else '/home/nu/catkin_ws_ov/maps/house_direct')
