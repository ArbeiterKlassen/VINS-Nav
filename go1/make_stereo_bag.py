#!/usr/bin/env python3
"""make_stereo_bag.py — 把双目 PNG 对（+可选 IMU CSV）打包成 rosbag（供离线 SLAM）。

用法（先 source ROS 环境）:
  source /opt/ros/noetic/setup.bash
  python3 make_stereo_bag.py --dir ~/VINS-Nav/go1/cam_evidence/stereo_local \
      --fps 10 --out /tmp/go1_stereo_test.bag [--imu-csv /tmp/low_imu.csv]

输入命名（自动探测其一）:
  left_*.png / right_*.png
  rect_left_*.png / rect_right_*.png
  raw_left_*.png / raw_right_*.png

输出 topics:
  /go1/cam0/image_raw, /go1/cam1/image_raw,
  /go1/cam0/camera_info, /go1/cam1/camera_info,
  /go1/imu（若提供 lowstate_rx 的 CSV）
内参为估计值（hFov≈90° ⇒ 928 宽时 fx=fy=464；按实际宽度缩放），后续用标定替换。
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
import time

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu

PAIR_GLOBS = [
    ("left_*.png", "right_*.png"),
    ("rect_left_*.png", "rect_right_*.png"),
    ("raw_left_*.png", "raw_right_*.png"),
]


def find_pairs(d: str):
    for lg, rg in PAIR_GLOBS:
        ls = sorted(glob.glob(os.path.join(d, lg)))
        rs = sorted(glob.glob(os.path.join(d, rg)))
        if ls and len(ls) == len(rs):
            return list(zip(ls, rs))
    return []


def cam_info(width: int, height: int, baseline: float, frame_id: str, stamp) -> CameraInfo:
    ci = CameraInfo()
    fi = float(width) / 928.0
    fx = 464.0 * fi
    fy = 464.0 * fi
    cx = width / 2.0
    cy = height / 2.0
    ci.width, ci.height = width, height
    ci.distortion_model = "plumb_bob"
    ci.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    ci.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    tx = -fx * baseline if frame_id == "cam1" else 0.0
    ci.P = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.header.stamp = stamp
    ci.header.frame_id = frame_id
    return ci


def img_msg(im: np.ndarray, frame_id: str, stamp) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = im.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = im.tobytes()
    return msg


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dir", required=True, help="PNG 对所在目录")
    ap.add_argument("--fps", type=float, default=10.0, help="无时间戳时的标称帧率")
    ap.add_argument("--out", default="/tmp/go1_stereo.bag")
    ap.add_argument("--imu-csv", default="", help="lowstate_rx --csv 生成的 CSV")
    ap.add_argument("--baseline", type=float, default=0.02509)
    ap.add_argument("--repeat", type=int, default=1,
                    help="把帧对循环重复 N 次（冒烟/拉长序列用）")
    args = ap.parse_args()

    pairs = find_pairs(args.dir)
    if not pairs:
        print(f"no stereo pairs found in {args.dir}", file=sys.stderr)
        return 1

    # 时间基准: 有 IMU CSV 时与之对齐（首条 IMU 时间），否则用当前墙钟。
    # 不能用 epoch 1.0：TF2 按墙钟裁剪缓存，1970 的时间戳会导致
    # "Lookup would require extrapolation ... into the past" 而丢掉数据。
    t_start = time.time()
    if args.imu_csv:
        with open(args.imu_csv, newline="") as f:
            for row in csv.DictReader(f):
                try:
                    t_start = float(row["t_wall"])
                    break
                except (KeyError, ValueError):
                    continue

    n_written = 0
    with rosbag.Bag(args.out, "w") as bag:
        for i in range(len(pairs) * args.repeat):
            lp, rp = pairs[i % len(pairs)]
            l_img = cv2.imread(lp)
            r_img = cv2.imread(rp)
            if l_img is None or r_img is None:
                continue
            stamp = rospy.Time.from_sec(t_start + i / args.fps)
            bag.write("/go1/cam0/image_raw", img_msg(l_img, "cam0", stamp), stamp)
            bag.write("/go1/cam1/image_raw", img_msg(r_img, "cam1", stamp), stamp)
            bag.write("/go1/cam0/camera_info",
                      cam_info(l_img.shape[1], l_img.shape[0], args.baseline, "cam0", stamp), stamp)
            bag.write("/go1/cam1/camera_info",
                      cam_info(r_img.shape[1], r_img.shape[0], args.baseline, "cam1", stamp), stamp)
            n_written += 1

        n_imu = 0
        if args.imu_csv:
            with open(args.imu_csv, newline="") as f:
                for row in csv.DictReader(f):
                    try:
                        stamp = rospy.Time.from_sec(float(row["t_wall"]))
                        msg = Imu()
                        msg.header.stamp = stamp
                        msg.header.frame_id = "imu"
                        msg.orientation.x = float(row["qx"])
                        msg.orientation.y = float(row["qy"])
                        msg.orientation.z = float(row["qz"])
                        msg.orientation.w = float(row["qw"])
                        msg.angular_velocity.x = float(row["gx"])
                        msg.angular_velocity.y = float(row["gy"])
                        msg.angular_velocity.z = float(row["gz"])
                        msg.linear_acceleration.x = float(row["ax"])
                        msg.linear_acceleration.y = float(row["ay"])
                        msg.linear_acceleration.z = float(row["az"])
                        bag.write("/go1/imu", msg, stamp)
                        n_imu += 1
                    except (KeyError, ValueError):
                        continue
        print(f"wrote {n_written} stereo pairs, {n_imu} imu samples -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
