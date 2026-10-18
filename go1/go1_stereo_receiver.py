#!/usr/bin/env python3
"""go1_stereo_receiver.py — 接收 .13 的 TCP-JPEG 双目流（Go1 face 相机）。

配合 stage/stereo_tcp_sender.cc（运行在 .13，/dev/shm 编译）。
协议（发送端定义）：33B 头 + JPEG：
  magic "G1ST"(4) | eye u8 (0=L,1=R) | frame_id u64 | ts_us u64
  | width u32 | height u32 | jpeg_len u32 | jpeg

功能（可选组合）：
  --ros            发布 /go1/cam0/image_raw, /go1/cam1/image_raw,
                   /go1/cam0/camera_info, /go1/cam1/camera_info
  --save-dir DIR   按帧存 PNG（--save-every N 控制频率；默认每 10 对存 1 对）
  --rotate-saved   存盘图旋转 180°（校正图是倒的，仅供人看；ROS 图像保持原始方向）

时间戳：左右目同帧共享同一 ROS stamp（接收时刻）。
相机内参默认按 928x800/眼、hFov≈90° 估计：fx=fy=464, cx=464, cy=400；
若实际分辨率不同按宽度比例缩放；--baseline 默认 0.02509 m（出厂 Translation）。
注意：这是估计值，后续用棋盘格标定替换。
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time

import numpy as np
import cv2

HDR = struct.Struct("<4sBQQIII")
MAGIC = b"G1ST"


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf.extend(chunk)
    return bytes(buf)


def make_camera_info(width: int, height: int, baseline_m: float, frame_id: str):
    from sensor_msgs.msg import CameraInfo

    ci = CameraInfo()
    fi = float(width) / 928.0  # 估计内参按 928 宽基准缩放
    fx = 464.0 * fi
    fy = 464.0 * fi
    cx = width / 2.0
    cy = height / 2.0
    ci.width = width
    ci.height = height
    ci.distortion_model = "plumb_bob"
    ci.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    ci.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # cam1（右目）投影矩阵带 -fx*baseline
    tx = -fx * baseline_m if frame_id == "cam1" else 0.0
    ci.P = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.header.frame_id = frame_id
    return ci


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=9101)
    ap.add_argument("--ros", action="store_true", help="发布 ROS 图像/相机信息")
    ap.add_argument("--save-dir", default="", help="保存 PNG 对的目录（空=不存）")
    ap.add_argument("--save-every", type=int, default=10, help="每 N 对存 1 对")
    ap.add_argument("--rotate-saved", action="store_true", default=True)
    ap.add_argument("--no-rotate-saved", action="store_false", dest="rotate_saved")
    ap.add_argument("--baseline", type=float, default=0.02509, help="米（出厂估计）")
    ap.add_argument("--topic-prefix", default="/go1")
    args = ap.parse_args()

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    pubs = None
    if args.ros:
        import rospy
        from sensor_msgs.msg import Image

        rospy.init_node("go1_stereo_receiver", anonymous=False)
        pubs = {
            "img0": rospy.Publisher(f"{args.topic_prefix}/cam0/image_raw", Image, queue_size=2),
            "img1": rospy.Publisher(f"{args.topic_prefix}/cam1/image_raw", Image, queue_size=2),
            "ci0": rospy.Publisher(f"{args.topic_prefix}/cam0/camera_info", type(make_camera_info(928, 800, args.baseline, "cam0")), queue_size=2),
            "ci1": rospy.Publisher(f"{args.topic_prefix}/cam1/camera_info", type(make_camera_info(928, 800, args.baseline, "cam1")), queue_size=2),
        }
        print(f"[ros] publishing {args.topic_prefix}/cam{{0,1}}/image_raw + camera_info")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"[net] listening on 0.0.0.0:{args.port} ...")
    sys.stdout.flush()

    pairs = 0
    saved = 0
    bytes_total = 0
    t0 = time.time()
    tprev = t0

    while True:
        conn, addr = srv.accept()
        print(f"[net] sender connected: {addr}")
        sys.stdout.flush()
        pending: dict[int, dict[int, tuple]] = {}
        try:
            while True:
                hdr = recv_exact(conn, HDR.size)
                magic, eye, fid, ts_us, w, h, ln = HDR.unpack(hdr)
                if magic != MAGIC:
                    raise ConnectionError(f"bad magic {magic!r}")
                data = recv_exact(conn, ln)
                bytes_total += HDR.size + ln
                img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                slot = pending.setdefault(fid, {})
                slot[eye] = (img, ts_us)
                if len(slot) < 2:
                    continue
                l_img = slot[0][0]
                r_img = slot[1][0]
                ts = slot[0][1]
                del pending[fid]
                if len(pending) > 8:
                    pending.clear()

                pairs += 1
                stamp = None
                if pubs:
                    import rospy
                    from sensor_msgs.msg import Image

                    stamp = rospy.Time.from_sec(time.time())
                    for key, name, im in (("img0", "cam0", l_img), ("img1", "cam1", r_img)):
                        msg = Image()
                        msg.header.stamp = stamp
                        msg.header.frame_id = name
                        msg.height, msg.width = im.shape[:2]
                        msg.encoding = "bgr8"
                        msg.is_bigendian = 0
                        msg.step = msg.width * 3
                        msg.data = im.tobytes()
                        pubs[key].publish(msg)
                    for key, name in (("ci0", "cam0"), ("ci1", "cam1")):
                        ci = make_camera_info(l_img.shape[1], l_img.shape[0], args.baseline, name)
                        ci.header.stamp = stamp
                        pubs[key].publish(ci)

                if args.save_dir and args.save_every > 0 and pairs % args.save_every == 0:
                    if args.rotate_saved:
                        l_s = cv2.rotate(l_img, cv2.ROTATE_180)
                        r_s = cv2.rotate(r_img, cv2.ROTATE_180)
                    else:
                        l_s, r_s = l_img, r_img
                    cv2.imwrite(os.path.join(args.save_dir, f"left_{saved:04d}.png"), l_s)
                    cv2.imwrite(os.path.join(args.save_dir, f"right_{saved:04d}.png"), r_s)
                    saved += 1

                if pairs % 20 == 0:
                    now = time.time()
                    dt = now - tprev
                    tprev = now
                    print(f"[stats] pairs={pairs} rate={20.0 / dt:.2f} pairs/s "
                          f"bytes={bytes_total / 1e6:.2f} MB "
                          f"frame={l_img.shape[1]}x{l_img.shape[0]} saved={saved}")
                    sys.stdout.flush()
        except (ConnectionError, OSError) as e:
            print(f"[net] connection ended: {e}")
            sys.stdout.flush()
        finally:
            conn.close()


if __name__ == "__main__":
    sys.exit(main())
