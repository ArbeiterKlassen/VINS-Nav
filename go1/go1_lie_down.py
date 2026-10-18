#!/usr/bin/env python3
"""go1_lie_down.py — 让 Go1 卧倒（HighCmd mode 5；运动指令，谨慎使用）。

与 go1_stand_up.py 对应：
  mode 5 = position stand down（推荐的卧倒方式）；
  mode 7 = damping（直接卸力，会瘫软，作为备选）。
成功判据：bodyHeight < 0.15 m（站立约 0.28-0.31 m）。

⚠️ 安全要求（SOP）：狗在地面/净空、有人看护、手放电源；加 --confirm 才会发送。

用法:
  python3 go1_lie_down.py --dry-run
  python3 go1_lie_down.py --confirm                # mode 5
  python3 go1_lie_down.py --confirm --mode 7       # damping（瘫软）
  python3 go1_lie_down.py --confirm --hold 4

协议与 CRC 与 go1_stand_up.py 相同（已用官方 SDK 静态库三组向量逐字节校验）。
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys

from go1_stand_up import BODY_STAND_THRESHOLD, DST, build_cmd, parse_state, run_phase
from go1_stand_up import CRC_OFF


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mode", choices=["5", "7"], default="5")
    ap.add_argument("--confirm", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--pulse", type=float, default=2.5)
    ap.add_argument("--hold", type=float, default=4.0)
    args = ap.parse_args()

    mode = int(args.mode)
    frames = {mode: build_cmd(mode), 0: build_cmd(0)}

    if args.dry_run:
        f = frames[mode]
        print(f"mode={mode} len={len(f)} crc={struct.unpack_from('<I', f, CRC_OFF)[0]:#010x}")
        print("  " + f.hex())
        return 0

    if not args.confirm:
        print("⚠️  运动指令脚本：确认安全条件后加 --confirm；或先用 --dry-run 检查帧。")
        return 2

    print(f"⚠️  开始发送卧倒指令（mode {mode}）。保持看护。")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.connect(DST)
    sock.setblocking(False)

    state = None
    sent = recv = 0
    try:
        state, s, r = run_phase(sock, frames[mode], args.pulse, args.rate, f"mode{mode}", state)
        sent += s; recv += r
        state, s, r = run_phase(sock, frames[0], args.hold * 0.5, args.rate, "hold", state)
        sent += s; recv += r
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        sock.close()

    print("=== summary ===")
    print(f"sent={sent} recv={recv}")
    if state:
        lying = state["bodyHeight"] < BODY_STAND_THRESHOLD
        print(f"final: mode={state['mode']} bodyHeight={state['bodyHeight']:+.3f} m "
              f"→ {'LYING' if lying else 'NOT lying'}")
        return 0 if lying else 1
    print("no HighState received — 检查 8082 服务/其它客户端占用")
    return 2


if __name__ == "__main__":
    sys.exit(main())
