#!/usr/bin/env python3
"""一次性 /high_cmd 镜像发布 —— 触发 /high_state 重发布并采样。

背景（2026-10-11 实测）
----------------------
`unitree_legged_real/ros_udp` 只在收到 `/high_cmd` 的回调里发布 `/high_state`
（发布语句在回调内，而不是 500Hz 接收循环里）。因此 /high_state 平时"无数据"，
想看一眼实时高层状态必须发一条 /high_cmd。

本脚本发出的消息与 ros_udp 自启动以来持续发送的初始化指令**逐位相同**
（head=[0xFE,0xEF], levelFlag=0xEE, mode=0, 其余全零），因此对机器人不产生
任何行为变化；它只是把同一条指令再发一次，并触发一次状态重发布。

用法（本地 roscore，默认 ROS_MASTER_URI=http://localhost:11311）
----
    # 终端 1：先订阅，否则会错过那条消息
    source /opt/ros/noetic/setup.bash && source ~/go1_ws/devel/setup.bash
    rostopic echo -n1 /high_state
    # 终端 2：
    python3 pub_high_cmd_mirror_once.py

安全：仅高层 mirror 指令，不做任何速度/位置/模式变更。若未来要真正控制
（例如站起来 mode 6 / mode 8），属于运动指令，必须先与用户打招呼并有人看护。
"""

from __future__ import annotations

import argparse
import sys
import time

import rospy
from unitree_legged_msgs.msg import BmsCmd, HighCmd, LED


def build_mirror_cmd() -> HighCmd:
    m = HighCmd()
    m.head = [0xFE, 0xEF]
    m.levelFlag = 0xEE
    m.frameReserve = 0
    m.SN = [0, 0]
    m.version = [0, 0]
    m.bandWidth = 0
    m.mode = 0          # idle / default stand —— 与 ros_udp 启动后持续发送的值一致
    m.gaitType = 0
    m.speedLevel = 0
    m.footRaiseHeight = 0.0
    m.bodyHeight = 0.0
    m.position = [0.0, 0.0]
    m.euler = [0.0, 0.0, 0.0]
    m.velocity = [0.0, 0.0]
    m.yawSpeed = 0.0
    m.bms = BmsCmd()
    m.led = [LED(), LED(), LED(), LED()]
    m.wirelessRemote = [0] * 40
    m.reserve = 0
    m.crc = 0
    return m


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--count", type=int, default=2, help="发布次数（默认 2）")
    args = ap.parse_args()

    rospy.init_node("oneshot_mirror_cmd", anonymous=True)
    pub = rospy.Publisher("/high_cmd", HighCmd, queue_size=1)
    time.sleep(1.2)  # 等连接建立
    msg = build_mirror_cmd()
    for _ in range(args.count):
        pub.publish(msg)
        time.sleep(0.5)
    print(f"published mirror HighCmd x{args.count}: head=[0xFE,0xEF] "
          f"levelFlag=0xEE mode=0 all-else-zero")
    return 0


if __name__ == "__main__":
    sys.exit(main())
