#!/usr/bin/env python3
"""go1_remote_monitor.py — 监听 8007 LowState 里的手柄数据，定位 wirelessRemote 并解码按键。

手柄协议（unitree_legged_sdk/joystick.h，40B 结构）：
  +0  head[2]
  +2  uint16 btn（小端）: bit0=R1, 1=L1, 2=start, 3=select, 4=R2, 5=L2, 6=F1, 7=F2,
                          8=A, 9=B, 10=X, 11=Y, 12=up, 13=right, 14=down, 15=left
  +4  lx f32, +8 rx, +12 ry, +16 L2, +20 ly, +24 idle[16]
**R1+A = 0x0101。**

线上帧中该 40B 块的偏移未知（候选 759..798，tick@755 之后）。本工具持续轮询
8007，记录 700..819 区间的逐帧变化（排除 tick@755-758 与 crc@803-806），供离线
定位"按键时才会变的字节"。

用法:
  python3 go1_remote_monitor.py [--seconds 90] [--out /tmp/remote_monitor.bin]
安全: 只发送 64B 'x' 填充触发包（已获授权），不接触 8082。
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

from lowstate_rx import POLL

DST = ("192.168.123.10", 8007)
WATCH_LO, WATCH_HI = 700, 820
KNOWN_ALWAYS = set(range(755, 759)) | set(range(803, 807))  # tick / crc 每帧变化


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--hz", type=float, default=500.0)
    ap.add_argument("--out", default="/tmp/remote_monitor.bin")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("", 0))
    sock.setblocking(False)
    f = open(args.out, "wb")

    period = 1.0 / args.hz
    t0 = time.time()
    next_send = t0
    t_report = t0
    sent = recv = 0
    prev = None
    ever: dict[int, int] = {}
    printed = 0
    print(f"[remote] listening {args.seconds:g}s @{args.hz:g}Hz -> {args.out}")
    sys.stdout.flush()

    try:
        while time.time() - t0 < args.seconds:
            now = time.time()
            while next_send <= now:
                try:
                    sock.sendto(POLL, DST)
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
                if addr[0] != DST[0] or len(d) not in (820, 870):
                    continue
                recv += 1
                f.write(struct.pack("<Q", int(time.time() * 1e6)))
                f.write(d[WATCH_LO:WATCH_HI])
                seg = d[WATCH_LO:WATCH_HI]
                if prev is not None:
                    changed = []
                    for i in range(WATCH_LO, WATCH_HI):
                        a = prev[i - WATCH_LO]
                        b = seg[i - WATCH_LO]
                        if a != b and i not in KNOWN_ALWAYS:
                            changed.append((i, a, b))
                            ever[i] = ever.get(i, 0) + 1
                    if changed and printed < 600:
                        print("t=%.3f changed %s" % (time.time() - t0, changed[:12]))
                        sys.stdout.flush()
                        printed += 1
                prev = seg

            if time.time() - t_report >= 2.0:
                t_report = time.time()
                print("[remote] t=%.0fs recv=%d rx=%.0f/s ever_changed=%s" % (
                    t_report - t0, recv, recv / max(1e-3, t_report - t0),
                    sorted(ever.keys())[:48]))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        sock.close()

    print("[remote] done recv=%d" % recv)
    print("[remote] ever-changed offsets:", sorted(ever.keys()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
