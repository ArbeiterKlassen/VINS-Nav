#!/usr/bin/env python3
"""go1_stand_lie_cycle.py — 站立↔卧倒循环测试（运动指令，谨慎使用）。

每个循环：先 stand（mode 6，验证 bodyHeight ≥ 0.15）、再 lie（mode 5，验证 < 0.15）。
默认从"站立"开始（也满足"先测站立"的要求；当前已站立时该相位为幂等验证）。

⚠️ 安全要求（SOP）：狗在地面/净空、有人看护、手放电源开关；加 --confirm 才会发送。

用法:
  python3 go1_stand_lie_cycle.py --dry-run
  python3 go1_stand_lie_cycle.py --confirm --cycles 2
  python3 go1_stand_lie_cycle.py --confirm --cycles 3 --mode-lie 5 --pulse 2.5 --hold 4

协议/CRC 与 go1_stand_up.py 相同（官方 SDK 静态库三组向量逐字节校验）。
"""
from __future__ import annotations

import argparse
import socket
import sys

from go1_stand_up import BODY_STAND_THRESHOLD, DST, build_cmd, run_phase


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--confirm", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--mode-lie", choices=["5", "7"], default="5")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--pulse", type=float, default=2.5)
    ap.add_argument("--hold", type=float, default=4.0)
    args = ap.parse_args()

    lie_mode = int(args.mode_lie)
    frames = {6: build_cmd(6), lie_mode: build_cmd(lie_mode), 0: build_cmd(0)}

    if args.dry_run:
        for m in (6, lie_mode):
            print(f"mode={m}: {frames[m].hex()}")
        return 0

    if not args.confirm:
        print("⚠️  这是运动指令循环测试脚本：确认安全条件后加 --confirm。")
        return 2

    print(f"⚠️  开始 {args.cycles} 个循环（stand=6 → lie={lie_mode}），保持看护。")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.connect(DST)
    sock.setblocking(False)

    state = None
    results = []
    try:
        for cyc in range(1, args.cycles + 1):
            print(f"\n=== cycle {cyc}/{args.cycles}: STAND (mode 6) ===")
            state, _, _ = run_phase(sock, frames[6], args.pulse, args.rate, "stand", state)
            state, _, _ = run_phase(sock, frames[0], args.hold * 0.5, args.rate, "hold", state)
            ok_stand = state is not None and state["bodyHeight"] >= BODY_STAND_THRESHOLD
            h_stand = state["bodyHeight"] if state else float("nan")
            print(f"  -> {'PASS' if ok_stand else 'FAIL'} (bodyHeight={h_stand:+.3f})")
            results.append((cyc, "stand", ok_stand, h_stand))

            print(f"=== cycle {cyc}/{args.cycles}: LIE (mode {lie_mode}) ===")
            state, _, _ = run_phase(sock, frames[lie_mode], args.pulse, args.rate, "lie", state)
            state, _, _ = run_phase(sock, frames[0], args.hold * 0.5, args.rate, "hold", state)
            ok_lie = state is not None and state["bodyHeight"] < BODY_STAND_THRESHOLD
            h_lie = state["bodyHeight"] if state else float("nan")
            print(f"  -> {'PASS' if ok_lie else 'FAIL'} (bodyHeight={h_lie:+.3f})")
            results.append((cyc, "lie", ok_lie, h_lie))

            if not (ok_stand and ok_lie):
                print("  异常：本循环未全部通过，停止后续循环")
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        sock.close()

    print("\n=== summary ===")
    for cyc, phase, ok, h in results:
        print(f"  cycle {cyc} {phase:5s}: {'PASS' if ok else 'FAIL'}  bodyHeight={h:+.3f} m")
    all_ok = all(ok for _, _, ok, _ in results)
    print("VERDICT:", "PASS" if all_ok else "FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
