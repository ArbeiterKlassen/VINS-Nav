#!/usr/bin/env python3
"""go1_imu_publisher.py — 8007 低层 IMU → ROS `/go1/imu`（500 Hz 轮询）。

用途：与双目桥（`go1_stereo_receiver.py --ros`）一起录制，得到含 IMU 的 rosbag，
供 OpenVINS / 离线分析使用。**用户已确认启用 8007 轮询。**

安全边界：
  * 只发送 64B `'x'` 填充触发包（非法 LowCmd，无控制副作用），机器人固件对它的
    处理是"回一帧状态、忽略内容"；
  * 不接触 `.161:8082`；录制期间请勿启动任何 8082 客户端（手柄需要独占
    sport 的客户端语义）。

用法（先 source ROS 环境）:
  source /opt/ros/noetic/setup.bash
  python3 go1_imu_publisher.py [--hz 500] [--csv /tmp/imu.csv] [--topic /go1/imu]

消息：orientation = LowState 四元数 (w,x,y,z)；angular_velocity = 陀螺 (rad/s)；
linear_acceleration = 加速度 (m/s²)；时间戳取发布时刻（墙钟，与图像同基准）。
"""
from __future__ import annotations

import argparse
import csv
import os
import socket
import sys
import time

import rospy
from sensor_msgs.msg import Imu

from lowstate_rx import POLL, parse_lowstate


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hz", type=float, default=500.0)
    ap.add_argument("--csv", default="", help="同时写 CSV（make_stereo_bag.py 可直接用）")
    ap.add_argument("--topic", default="/go1/imu")
    ap.add_argument("--dst", default=os.environ.get("GO1_IMU_DST", "192.168.123.10:8007"),
                    help="8007 目标 host:port（默认 .10:8007；测试可用本机假服务器）")
    args = ap.parse_args()
    host, _, port = args.dst.partition(":")
    dst = (host, int(port))

    rospy.init_node("go1_imu_publisher", anonymous=False)
    pub = rospy.Publisher(args.topic, Imu, queue_size=200)

    csvf = writer = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        writer = csv.writer(csvf)
        writer.writerow(["t_wall", "tick", "qw", "qx", "qy", "qz",
                         "gx", "gy", "gz", "ax", "ay", "az", "roll", "pitch", "yaw"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("", 0))
    sock.setblocking(False)

    period = 1.0 / args.hz
    next_send = time.time()
    t0 = next_send
    t_report = t0
    sent = recv = 0
    last_acc = (0.0, 0.0, 0.0)
    last_tick = None   # 按 tick 去重（实测 .10 会对同一请求回两个包）

    print(f"[imu] polling {dst[0]}:{dst[1]} at {args.hz:g} Hz -> {args.topic}")
    sys.stdout.flush()
    try:
        while not rospy.is_shutdown():
            now = time.time()
            while next_send <= now:
                try:
                    sock.sendto(POLL, dst)
                    sent += 1
                except OSError:
                    pass
                next_send += period
            dt = next_send - time.time()
            if dt > 0:
                time.sleep(dt)

            for _ in range(64):
                try:
                    d, addr = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except OSError:
                    break
                recv += 1
                if addr[0] != dst[0]:
                    continue          # 只接受目标主机的回包
                try:
                    fr = parse_lowstate(d)
                except ValueError:
                    continue
                if fr["tick"] == last_tick:
                    continue          # 去重（服务会对同一请求回重复包）
                last_tick = fr["tick"]
                imu = fr["imu"]
                q = imu["quaternion"]
                g = imu["gyroscope"]
                a = imu["accelerometer"]
                rpy = imu["rpy"]
                stamp = rospy.Time.now()
                msg = Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = "imu"
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q[1], q[2], q[3], q[0]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
                msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = 1e-3
                msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 1e-4
                msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = 1e-2
                pub.publish(msg)
                last_acc = a
                if writer:
                    writer.writerow([f"{now:.6f}", fr["tick"], q[0], q[1], q[2], q[3],
                                     g[0], g[1], g[2], a[0], a[1], a[2],
                                     rpy[0], rpy[1], rpy[2]])

            if time.time() - t_report >= 2.0:
                t_report = time.time()
                an = (last_acc[0] ** 2 + last_acc[1] ** 2 + last_acc[2] ** 2) ** 0.5
                print(f"[imu] t={t_report - t0:6.1f}s sent={sent} recv={recv} "
                      f"rx={recv / max(1e-3, t_report - t0):.0f}/s |a|={an:.2f}")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if csvf:
            csvf.close()
    print(f"[imu] stopped: sent={sent} recv={recv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
