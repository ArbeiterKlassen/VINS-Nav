#!/usr/bin/env python3
"""go1_stand_up.py — 让 Go1 站起来（HighCmd mode 6 / 8；运动指令，谨慎使用）。

背景（2026-10-11 实机）：机器人处于 damping（mode 7）时，mode 0（idle）不会
使它站起；候选指令是 HighCmd.mode = 6（position stand up）或 8（recovery stand）。

⚠️ 安全要求（项目 SOP，必须先满足）：
  * 狗在地面、四周 ≥2m 净空、有人看护、手放在电源开关上；
  * 使用前已与用户打招呼；
  * 必须显式加 --confirm 才会真正发包（否则只打印将要发送的帧）。

用法:
  python3 go1_stand_up.py --dry-run          # 构造并打印 129B 帧（不发包）
  python3 go1_stand_up.py --confirm          # auto: 先 mode 6；未站起再试 mode 8
  python3 go1_stand_up.py --confirm --mode 6
  python3 go1_stand_up.py --confirm --mode 8 --hold 5

协议（与 unitree_legged_sdk 字节级一致；CRC 已用官方 amd64 静态库三组向量校准）：
  HighCmd 129B packed: head FE EF, levelFlag 0xEE, mode=6/8, 其余 0。
  CRC = 对前 124 字节按 31 个小端 u32 逐字 MSB-first 迭代
        (poly 0x04C11DB7, init 0xFFFFFFFF, 无最终异或)，写入偏移 125。
  目标 192.168.123.161:8082（sport 单客户端服务），默认 500 Hz；服务对每个包
  回 1087B HighState（mode@885, bodyHeight@907, IMU@22）。
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

DST = ("192.168.123.161", 8082)
CMD_LEN = 129
CRC_OFF = 125
POLY = 0x04C11DB7
HIGHSTATE_LEN = 1087
BODY_STAND_THRESHOLD = 0.15  # m; 站立约 0.28-0.31，damping/躺平 ≈ 0.0-0.1


def unitree_crc32(buf124: bytes) -> int:
    """与 SDK UNITREE_LEGGED_SDK::crc32(uint32_t*, 31) 逐位一致的实现。"""
    crc = 0xFFFFFFFF
    for w in struct.unpack_from("<31I", buf124, 0):
        crc ^= w
        for _ in range(32):
            crc = (((crc << 1) ^ POLY) if (crc & 0x80000000) else (crc << 1)) & 0xFFFFFFFF
    return crc


def build_cmd(mode: int) -> bytes:
    b = bytearray(CMD_LEN)
    b[0] = 0xFE
    b[1] = 0xEF
    b[2] = 0xEE          # levelFlag = HIGHLEVEL
    b[3] = 0x00          # frameReserve
    b[22] = mode         # HighCmd.mode
    crc = unitree_crc32(bytes(b[:124]))
    struct.pack_into("<I", b, CRC_OFF, crc)
    return bytes(b)


def parse_state(d: bytes) -> dict:
    if len(d) != HIGHSTATE_LEN or d[2] != 0xEE:
        raise ValueError(f"not a HighState: len={len(d)} lvl={d[2]:#x}")
    mode = d[885]
    body_height = struct.unpack_from("<f", d, 907)[0]
    quat = struct.unpack_from("<4f", d, 22)
    accel = struct.unpack_from("<3f", d, 50)
    return {"mode": mode, "bodyHeight": body_height, "quat": quat, "accel": accel}


def run_phase(sock, cmd_frame: bytes, seconds: float, rate: float, label: str,
              state: dict | None) -> tuple[dict | None, int, int]:
    """按 rate 发送 cmd_frame，持续 seconds 秒；返回 (最新状态, 发送数, 收到数)。

    Fail-fast：开始发送 1 秒内没有收到任何**有效** HighState，或连续 1 秒没有新的
    有效 HighState，立即停止发送（避免在运动服务异常时继续盲发指令）。
    """
    sent = recv = 0
    period = 1.0 / rate
    t0 = time.time()
    next_t = t0
    t_report = t0
    last_rx = None
    while time.time() - t0 < seconds:
        now = time.time()
        while next_t <= now:
            try:
                sock.send(cmd_frame)
                sent += 1
            except OSError as e:
                print(f"  send error: {e}")
                return state, sent, recv
            next_t += period
        dt = next_t - time.time()
        if dt > 0:
            time.sleep(dt)
        for _ in range(32):
            try:
                d, _ = sock.recvfrom(4096)
                recv += 1
                try:
                    state = parse_state(d)
                    last_rx = time.time()
                except ValueError:
                    pass
            except BlockingIOError:
                break
            except OSError:
                break
        now = time.time()
        # fail-fast: 无有效回包时立即停发
        if sent > 50:
            if (last_rx is None and now - t0 > 1.0) or \
               (last_rx is not None and now - last_rx > 1.0):
                print(f"  [{label}] ABORT: 连续 >1s 未收到有效 HighState — 停止发送（防盲发）")
                sys.stdout.flush()
                break
        if now - t_report >= 0.5:
            t_report = now
            if state:
                q = state["quat"]
                qn = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5
                print(f"  [{label}] t={now - t0:4.1f}s mode={state['mode']} "
                      f"bodyHeight={state['bodyHeight']:+.3f} |q|={qn:.3f} "
                      f"accel_z={state['accel'][2]:+.2f}  tx={sent} rx={recv}")
            else:
                print(f"  [{label}] t={now - t0:4.1f}s — no HighState replies (another client?)")
            sys.stdout.flush()
    return state, sent, recv


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mode", choices=["6", "8", "auto"], default="auto")
    ap.add_argument("--confirm", action="store_true", help="确认执行（否则拒绝发送）")
    ap.add_argument("--dry-run", action="store_true", help="只构造并打印帧，不发包")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--pulse", type=float, default=2.5, help="站立指令持续秒数")
    ap.add_argument("--hold", type=float, default=4.0, help="随后 mode=0 保持秒数")
    args = ap.parse_args()

    frames = {6: build_cmd(6), 8: build_cmd(8), 0: build_cmd(0)}

    if args.dry_run:
        for m in (6, 8):
            f = frames[m]
            print(f"mode={m} len={len(f)} crc={struct.unpack_from('<I', f, CRC_OFF)[0]:#010x}")
            print("  " + f.hex())
        return 0

    if not args.confirm:
        print("⚠️  这是运动指令脚本。确认安全条件（狗在地面/净空/有人看护/手放电源）后，")
        print("    加 --confirm 才会真正发送；或先用 --dry-run 检查将要发送的帧。")
        return 2

    print("⚠️  开始发送站立指令。保持看护，随时准备断电。")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.connect(DST)
    sock.setblocking(False)

    state = None
    total_sent = total_recv = 0
    try:
        if args.mode in ("6", "auto"):
            print("[phase] mode 6 (position stand up)")
            state, s, r = run_phase(sock, frames[6], args.pulse, args.rate, "mode6", state)
            total_sent += s; total_recv += r
            state, s, r = run_phase(sock, frames[0], args.hold * 0.5, args.rate, "hold", state)
            total_sent += s; total_recv += r
        if args.mode == "8" or (args.mode == "auto" and
                                (state is None or state["bodyHeight"] < BODY_STAND_THRESHOLD)):
            print("[phase] mode 8 (recovery stand)")
            state, s, r = run_phase(sock, frames[8], args.pulse, args.rate, "mode8", state)
            total_sent += s; total_recv += r
            state, s, r = run_phase(sock, frames[0], args.hold * 0.5, args.rate, "hold", state)
            total_sent += s; total_recv += r
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        sock.close()

    print("=== summary ===")
    print(f"sent={total_sent} recv={total_recv}")
    if state:
        stood = state["bodyHeight"] >= BODY_STAND_THRESHOLD
        print(f"final: mode={state['mode']} bodyHeight={state['bodyHeight']:+.3f} m "
              f"→ {'STANDING' if stood else 'NOT standing'}")
        return 0 if stood else 1
    print("no HighState received at all — 检查是否有其他客户端占用 8082 / 服务是否在跑")
    return 2


if __name__ == "__main__":
    sys.exit(main())
