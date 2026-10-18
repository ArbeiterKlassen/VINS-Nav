#!/usr/bin/env python3
"""Go1 低层状态只读接收器 (192.168.123.10:8007, UDP)。

用途
----
以"轮询-回包"方式接收 Go1 运动控制板发出的 820 字节低层状态帧（线上为
官方 unitree_legged_sdk 的压缩 LowState 格式；本脚本按 2026-10-11 实测
逆向出的偏移解码，并对齐 comm.h 的 LowState 结构）。

安全边界（重要）
----------------
* 本脚本只发送 64 字节的 b'x'*64 触发包。该载荷不可能是合法 LowCmd
  （长度/CRC 均不符），控制板对它的处理是"回一帧状态、忽略内容"。
  不发送任何有效控制帧、不设置任何电机模式/力矩/位置。
* 实测：服务对触发包 1:1 回包（发送率≈回包率）；不发则不收，纯被动不可行
  （--passive 仅用于复现该结论）。

帧格式（820 B；2026-10-18 起实测也有 870 B 变体——前 820B 布局相同，尾部多 50B，
本工具兼容两种帧长；偏移为实测值，小端）
-----------------------------------
  0..1    head = FE EF
  2       levelFlag = FF (LOWLEVEL)
  3       frameReserve
  4..11   SN        u32 x2   （与 HighState.SN 实测一致）
  12..19  version   u32 x2   （与 HighState.version 实测一致）
  20..21  bandWidth u16      （取值与 HighState.bandWidth 不一致，待考）
  22..37  quaternion    f32 x4 (w,x,y,z)
  38..49  gyroscope     f32 x3 rad/s
  50..61  accelerometer f32 x3 m/s^2
  62..73  rpy           f32 x3 rad
  74      imu temperature i8
  75+32k  电机槽 k=0..19（Go1 用前 12 个）:
      +0     mode u8 (0x0A=伺服, 0x00=阻尼)
      +1..4  q  f32 rad
      +5..8  dq f32 rad/s
      +23    temperature i8
      （+9..+22、+24..+31 实测恒 0，未定点标）
  755..758 tick u32 ms（实测逐毫秒 +1；单位与结构体注释一致）
  759..798 wirelessRemote 40B（手柄数据；2026-10-18 实测定位：
            head@759 = 55 51；btn u16LE@761（bit0=R1, bit8=A → R1+A=0x0101）；
            lx/rx/ry/L2/ly f32 @763/767/771/775/779；idle 16B @783..798）
  799..802 reserve
  803..806 CRC（每帧变化）
  807..819 未解释尾部（勿依赖）

用法
----
  python3 lowstate_rx.py --seconds 5 --hz 500              # 500 Hz 轮询 5 秒
  python3 lowstate_rx.py --seconds 10 --hz 250 --csv /tmp/imu.csv
  python3 lowstate_rx.py --passive --seconds 5             # 只听不发（预期收不到）
"""

from __future__ import annotations

import argparse
import csv
import math
import socket
import struct
import sys
import time

DST = ("192.168.123.10", 8007)
POLL = b"x" * 64
FRAME_LEN = 820
FRAME_LENS = (820, 870)  # 实测两种帧长（重启后出现 870）：前 820B 布局相同，尾部多 50B

MOTOR0 = 75
MOTOR_STRIDE = 32
TICK_OFF = 755
REMOTE_OFF = 759      # wirelessRemote 40B 块的起始偏移（实测）
BTN_OFF = REMOTE_OFF + 2
AXES_OFF = REMOTE_OFF + 4
BUTTON_NAMES = ["R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
                "A", "B", "X", "Y", "up", "right", "down", "left"]


def _f32(d: bytes, off: int) -> float:
    return struct.unpack_from("<f", d, off)[0]


def parse_lowstate(d: bytes) -> dict:
    """解析 820B 低层帧；返回 dict。未知/可疑字段按偏移直接给出，不做臆测。"""
    if len(d) not in FRAME_LENS:
        raise ValueError(f"unexpected frame length {len(d)} (want one of {FRAME_LENS})")
    if d[0] != 0xFE or d[1] != 0xEF or d[2] != 0xFF:
        raise ValueError(f"not a LOWLEVEL frame: head={d[0]:02x}{d[1]:02x} lvl={d[2]:02x}")

    frame = {
        "levelFlag": d[2],
        "frameReserve": d[3],
        "SN": struct.unpack_from("<2I", d, 4),
        "version": struct.unpack_from("<2I", d, 12),
        "bandWidth": struct.unpack_from("<H", d, 20)[0],
        "imu": {
            "quaternion": struct.unpack_from("<4f", d, 22),
            "gyroscope": struct.unpack_from("<3f", d, 38),
            "accelerometer": struct.unpack_from("<3f", d, 50),
            "rpy": struct.unpack_from("<3f", d, 62),
            "temperature": struct.unpack_from("<b", d, 74)[0],
        },
        "motors": [],
        "tick": struct.unpack_from("<I", d, TICK_OFF)[0],
        "crc_raw": d[803:807].hex(),
    }
    for k in range(20):
        o = MOTOR0 + k * MOTOR_STRIDE
        frame["motors"].append(
            {
                "mode": d[o],
                "q": _f32(d, o + 1),
                "dq": _f32(d, o + 5),
                "temperature": struct.unpack_from("<b", d, o + 23)[0],
            }
        )
    return frame


def parse_remote(d: bytes) -> dict:
    """解析 820B 帧中的手柄数据（unitree_legged_sdk/joystick.h 布局）。

    返回 {btn: u16, buttons: {name: bool}, axes: {lx,rx,ry,L2,ly}}。
    R1+A 的组合值为 0x0101（btn bit0 | bit8）。
    """
    if len(d) not in FRAME_LENS:
        raise ValueError(f"unexpected frame length {len(d)}")
    btn = struct.unpack_from("<H", d, BTN_OFF)[0]
    lx, rx, ry, l2, ly = struct.unpack_from("<5f", d, AXES_OFF)
    return {
        "btn": btn,
        "buttons": {name: bool(btn & (1 << i)) for i, name in enumerate(BUTTON_NAMES)},
        "axes": {"lx": lx, "rx": rx, "ry": ry, "L2": l2, "ly": ly},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hz", type=float, default=500.0, help="轮询频率（发送触发包）")
    ap.add_argument("--seconds", type=float, default=5.0, help="采样时长")
    ap.add_argument("--passive", action="store_true", help="只听不发（预期收不到，用于复现）")
    ap.add_argument("--csv", default=None, help="可选：逐帧写入 CSV")
    ap.add_argument("--quiet", action="store_true", help="只在结束时打印汇总")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("", 0))  # 服务按来源端口回包，用临时端口即可
    sock.setblocking(False)

    writer = None
    csvf = None
    if args.csv:
        csvf = open(args.csv, "w", newline="")
        cols = ["t_wall", "tick", "qw", "qx", "qy", "qz",
                "gx", "gy", "gz", "ax", "ay", "az", "roll", "pitch", "yaw"]
        cols += [f"q{i}" for i in range(12)] + [f"dq{i}" for i in range(12)]
        writer = csv.writer(csvf)
        writer.writerow(cols)

    sent = 0
    recv = 0
    sizes: set[int] = set()
    errs = 0
    last: dict | None = None
    first_tick = None
    t0 = time.time()
    t_last_report = t0
    next_send = t0
    period = 0.0 if args.passive else 1.0 / args.hz

    print(f"[lowstate_rx] {'passive' if args.passive else f'{args.hz:g} Hz poll'} "
          f"-> {DST[0]}:{DST[1]}, {args.seconds:g}s")
    try:
        while True:
            now = time.time()
            if now - t0 >= args.seconds:
                break
            if not args.passive:
                while next_send <= now:
                    try:
                        sock.sendto(POLL, DST)
                        sent += 1
                    except OSError as e:
                        errs += 1
                        if errs <= 3:
                            print(f"  send error: {e}", file=sys.stderr)
                    next_send += period
                dt = next_send - time.time()
                if dt > 0:
                    time.sleep(dt)

            # 排空本 tick 到达的所有回包
            for _ in range(64):
                try:
                    d, _addr = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except OSError:
                    break
                recv += 1
                sizes.add(len(d))
                try:
                    fr = parse_lowstate(d)
                except ValueError as e:
                    errs += 1
                    if errs <= 3:
                        print(f"  parse error: {e}", file=sys.stderr)
                    continue
                if first_tick is None:
                    first_tick = fr["tick"]
                last = fr
                if writer:
                    imu, mo = fr["imu"], fr["motors"]
                    row = [f"{time.time():.6f}", fr["tick"], *imu["quaternion"],
                           *imu["gyroscope"], *imu["accelerometer"], *imu["rpy"]]
                    row += [mo[i]["q"] for i in range(12)] + [mo[i]["dq"] for i in range(12)]
                    writer.writerow(row)

            if not args.quiet and now - t_last_report >= 1.0:
                t_last_report = now
                if last is None:
                    print(f"  t={now - t0:6.1f}s  sent={sent}  recv=0  (无回包)")
                else:
                    a = last["imu"]["accelerometer"]
                    print(f"  t={now - t0:6.1f}s  sent={sent}  recv={recv}  "
                          f"tick={last['tick']}  |a|={math.sqrt(sum(x * x for x in a)):.2f}  "
                          f"yaw={math.degrees(last['imu']['rpy'][2]):6.1f}deg  "
                          f"m0.q={last['motors'][0]['q']:+.3f}")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if csvf:
            csvf.close()
        sock.close()

    wall = time.time() - t0
    print("=== summary ===")
    print(f"sent={sent} recv={recv} recv/sent={recv / sent if sent else 0:.3f} "
          f"wall={wall:.2f}s  rx_rate={recv / wall:.1f}/s  sizes={sorted(sizes)}")
    if last is None:
        print("VERDICT: FAIL — no frames received")
        if args.passive:
            print("(expected: the service replies to polls; it does not stream unsolicited)")
        return 1

    checks = []
    imu = last["imu"]
    qn = math.sqrt(sum(x * x for x in imu["quaternion"]))
    an = math.sqrt(sum(x * x for x in imu["accelerometer"]))
    checks.append(("frame size in {820,870}", set(sizes) <= set(FRAME_LENS)))
    checks.append(("quaternion norm ~1", abs(qn - 1.0) < 1e-3))
    checks.append(("|accel| in 8..12 m/s^2", 8.0 < an < 12.0))
    tick_rate = None
    if first_tick is not None and last["tick"] != first_tick:
        tick_rate = (last["tick"] - first_tick) / wall  # ms of tick per s of wall
        checks.append(("tick advances ~1000 ms/s", 900 < tick_rate < 1100))
    nmode = sum(1 for m in last["motors"][:12] if m["mode"] == 0x0A)
    print(f"last frame: tick={last['tick']}  quat_norm={qn:.6f}  |accel|={an:.2f}  "
          f"rpy=({imu['rpy'][0]:+.3f},{imu['rpy'][1]:+.3f},{imu['rpy'][2]:+.3f})  "
          f"motors servoed={nmode}/12")
    print(f"motor q : {[round(m['q'], 3) for m in last['motors'][:12]]}")
    print(f"motor dq: {[round(m['dq'], 3) for m in last['motors'][:12]]}")
    print(f"motor temp: {[m['temperature'] for m in last['motors'][:12]]}")
    if tick_rate is not None:
        print(f"tick_rate={tick_rate:.1f} ms/s")
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    ok_all = all(ok for _, ok in checks)
    print(f"VERDICT: {'PASS' if ok_all else 'CHECK'}")
    return 0 if ok_all else 2


if __name__ == "__main__":
    sys.exit(main())
