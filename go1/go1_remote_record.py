#!/usr/bin/env python3
"""go1_remote_record.py — 用遥控器 **R1+A** 控制「双目 + IMU」rosbag 录制。

工作方式：
  1. 启动（按需）roscore、双目接收端、IMU/手柄发布器（/go1/imu、/go1/remote）、
     `.13` 上的双目 TCP/JPEG 发送端；
  2. 等待数据就绪后，按 **R1+A 开始录制**，再按一次 **R1+A 停止并保存**；
     可重复多次录制（每次一个 bag）；Ctrl-C 退出并清理全部进程。
  3. 防抖：触发后必须**完整松开（btn==0）持续 ≥0.6 s** 才会重新武装，
     一次按压只切换一次；每次切换后另有 0.8 s 间隔锁。

安全：
  * 录制期间**不启动任何 8082 客户端**（sport 客户端语义留给手柄）；
  * 8007 轮询（IMU/手柄）已获用户确认，只发 64B 非法填充触发包；
  * 运动由手柄控制，请保持有人看护。

用法:
  source /opt/ros/noetic/setup.bash
  python3 go1_remote_record.py [--out-dir ~/go1_recordings] [--no-remote]
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

import rospy
from sensor_msgs.msg import Image, Joy

TOPICS = ["/go1/cam0/image_raw", "/go1/cam1/image_raw",
          "/go1/cam0/camera_info", "/go1/cam1/camera_info", "/go1/imu"]
R1A_MASK = (1 << 0) | (1 << 8)   # R1 + A = 0x0101
SENDER_IP = "192.168.123.13"


def master_running() -> bool:
    return subprocess.run(["pgrep", "-f", "rosmaster --core"],
                          stdout=subprocess.DEVNULL).returncode == 0


def spawn(cmd, log):
    os.makedirs(os.path.dirname(log), exist_ok=True)
    return subprocess.Popen(cmd, stdout=open(log, "ab"),
                            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)


def ssh(remote_cmd: str) -> None:
    subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
                    f"unitree@{SENDER_IP}", remote_cmd],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Recorder:
    def __init__(self, out_dir: str, prefix: str):
        self.out_dir = out_dir
        self.prefix = prefix
        self.proc = None
        self.path = None
        self.t0 = 0.0
        self.takes = []

    @property
    def active(self) -> bool:
        return self.proc is not None

    def start(self) -> None:
        os.makedirs(self.out_dir, exist_ok=True)
        name = "%s_%s.bag" % (self.prefix, time.strftime("%Y%m%d_%H%M%S"))
        self.path = os.path.join(self.out_dir, name)
        self.proc = subprocess.Popen(["rosbag", "record", "-O", self.path] + TOPICS,
                                     stdout=open("/tmp/rosbag_rec.log", "ab"),
                                     stderr=subprocess.STDOUT)
        self.t0 = time.time()
        print("\n[REC] ▶ START -> %s" % self.path, flush=True)

    def stop(self) -> None:
        if not self.proc:
            return
        try:
            self.proc.send_signal(signal.SIGINT)
            self.proc.wait(timeout=20)
        except Exception:
            self.proc.kill()
        dur = time.time() - self.t0
        size = os.path.getsize(self.path) if os.path.exists(self.path) else 0
        self.takes.append(self.path)
        print("[REC] ■ STOP  -> %s  (%.1f s, %.1f MB)" % (self.path, dur, size / 1e6),
              flush=True)
        print("[REC]   next: press R1+A to record another take; Ctrl-C to finish.",
              flush=True)
        self.proc = None


class Trigger:
    """R1+A 边沿检测 + 松开重武装防抖。"""

    def __init__(self, on_toggle):
        self.on_toggle = on_toggle
        self.armed = True
        self.zero_since = None
        self.last_toggle = 0.0

    def on_joy(self, msg: Joy) -> None:
        btn = 0
        for i, b in enumerate(msg.buttons[:16]):
            if b:
                btn |= (1 << i)
        now = time.time()
        if btn == 0:
            if self.zero_since is None:
                self.zero_since = now
            elif not self.armed and now - self.zero_since >= 0.6:
                self.armed = True
        else:
            self.zero_since = None
        if self.armed and (btn & R1A_MASK) == R1A_MASK and now - self.last_toggle > 0.8:
            self.armed = False
            self.last_toggle = now
            self.on_toggle()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", default=os.path.expanduser("~/go1_recordings"))
    ap.add_argument("--prefix", default="go1_rec")
    ap.add_argument("--no-remote", action="store_true", help="不启动 .13 上的发送端")
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    children = []
    started_master = False

    if not master_running():
        print("[session] starting roscore")
        children.append(spawn(["roscore"], "/tmp/roscore_remoterec.log"))
        started_master = True
        for _ in range(30):
            if master_running():
                break
            time.sleep(0.5)

    # 预清理（本地接收端 / 远端发送端）
    subprocess.run(["pkill", "-f", "go1_stereo_recei[v]er.py"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not args.no_remote:
        ssh('kill $(cat /dev/shm/ucsdk_run/sender.pid) 2>/dev/null; '
            'pkill -f "stereo_tcp_sende[r]" 2>/dev/null; true')

    print("[session] starting stereo receiver")
    children.append(spawn([sys.executable, os.path.join(script_dir, "go1_stereo_receiver.py"), "--ros"],
                          "/tmp/stereo_receiver.log"))
    time.sleep(1.5)
    print("[session] starting IMU/remote publisher (8007, 500Hz)")
    children.append(spawn([sys.executable, os.path.join(script_dir, "go1_imu_publisher.py"), "--hz", "500"],
                          "/tmp/imu_pub.log"))
    if not args.no_remote:
        print("[session] starting stereo sender on %s" % SENDER_IP)
        ssh('cd /dev/shm/ucsdk_run && (setsid nohup ./stereo_tcp_sender rect 192.168.123.101 9101 '
            '> sender.log 2>&1 < /dev/null & echo $! > sender.pid); sleep 1; true')

    rospy.init_node("go1_remote_record", anonymous=False)
    print("[session] waiting for /go1/remote ...")
    try:
        rospy.wait_for_message("/go1/remote", Joy, timeout=40)
        print("[session] waiting for stereo images ...")
        rospy.wait_for_message("/go1/cam1/image_raw", Image, timeout=40)
    except rospy.ROSException as e:
        print("[session] ERROR: %s" % e)
        print("  hints: camera free? (camera_nodes.sh) / sender log: ssh %s 'tail /dev/shm/ucsdk_run/sender.log'" % SENDER_IP)
        return 1

    rec = Recorder(args.out_dir, args.prefix)
    trig = Trigger(lambda: rec.stop() if rec.active else rec.start())
    sub = rospy.Subscriber("/go1/remote", Joy, trig.on_joy, queue_size=50)

    print("\n" + "=" * 62)
    print(" READY — 按遥控 **R1+A 开始录制**；再按一次 **R1+A 结束**")
    print(" 可重复多段；Ctrl-C 退出并清理。运动全程由手柄控制。")
    print("=" * 62 + "\n", flush=True)

    try:
        while not rospy.is_shutdown():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rec.active:
            rec.stop()
        try:
            sub.unregister()
        except Exception:
            pass
        for p in children:
            try:
                p.send_signal(signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        for p in children:
            try:
                p.kill()
            except Exception:
                pass
        if not args.no_remote:
            ssh('kill $(cat /dev/shm/ucsdk_run/sender.pid) 2>/dev/null; '
                'pkill -f "stereo_tcp_sende[r]" 2>/dev/null; true')
        if started_master:
            subprocess.run(["pkill", "-f", "rosmaster --core"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["pkill", "-f", "/opt/ros/noetic/bin/rosout"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("\n[session] takes:")
        for t in rec.takes:
            print("  ", t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
