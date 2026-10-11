#!/usr/bin/env python3
"""
udp_probe.py -- prove the Go1 camera streams actually arrive, without decoding.

The receiving example shipped with UnitreeCameraSDK (example_getimagetrans.cc)
builds this pipeline:

    udpsrc address=192.168.123.<SEG> port=<9201..9205>
      ! application/x-rtp,media=video,encoding-name=H264
      ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink

Two things make that unusable as a first test on the VM:

  * `omxh264dec` is the Jetson hardware decoder -- it does not exist on x86.
  * `h264parse` and `avdec_h264` (software) come from gstreamer1.0-plugins-bad
    and gstreamer1.0-libav, which are not installed here.

Neither matters for connectivity. This probe just binds the ports and reports
what arrives: packet counts, byte rates, and whether the payload looks like RTP.
If packets show up here, the network path is proven and decoding is a separate
problem; if nothing shows up, no amount of decoding work would have helped.

Port map, from the SDK comment (NOT the 9000+ scheme in StereoCameraCommon.hpp,
which is the SDK's own udpsink path -- the two are different mechanisms):

    9201 front    9202 chin    9203 left    9204 right    9205 abdomen

Usage:
    python3 udp_probe.py                     # all five ports, 60 s
    python3 udp_probe.py --ports 9201 9204   # just the pair we care about
    python3 udp_probe.py --seconds 0         # run until Ctrl-C
"""

import argparse
import select
import socket
import struct
import sys
import time

PORT_NAMES = {9201: 'front', 9202: 'chin', 9203: 'left', 9204: 'right', 9205: 'abdomen'}


def describe_rtp(payload):
    """Best-effort check that a payload looks like RTP/H264."""
    if len(payload) < 12:
        return 'too short'
    b0 = payload[0]
    version = b0 >> 6
    if version != 2:
        return f'not RTP (version={version})'
    pt = payload[1] & 0x7F
    seq = struct.unpack('>H', payload[2:4])[0]
    return f'RTP v2 pt={pt} seq={seq}'

    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ports', type=int, nargs='+',
                    default=[9201, 9202, 9203, 9204, 9205])
    ap.add_argument('--seconds', type=float, default=60.0,
                    help='0 runs until interrupted')
    ap.add_argument('--bind', default='0.0.0.0',
                    help='bind address; the sender targets 192.168.123.<SEG>')
    ap.add_argument('--report', type=float, default=2.0,
                    help='seconds between progress lines')
    args = ap.parse_args()

    socks = {}
    for p in args.ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((args.bind, p))
        except OSError as e:
            print(f"  port {p}: bind failed: {e}", file=sys.stderr)
            continue
        s.setblocking(False)
        socks[p] = s

    if not socks:
        print("no ports bound -- is something else using them?", file=sys.stderr)
        return 1

    print(f"listening on {args.bind} for {len(socks)} port(s): "
          f"{', '.join(str(p) for p in sorted(socks))}")
    for p in sorted(socks):
        name = PORT_NAMES.get(p, '?')
        print(f"  {p}  ({name})")
    print()

    stats = {p: {'n': 0, 'bytes': 0, 'first': None, 'last': None, 'sample': b''}
             for p in socks}
    t_start = time.time()
    t_report = t_start
    seen_any = False

    try:
        while True:
            now = time.time()
            if args.seconds and (now - t_start) >= args.seconds:
                break
            ready, _, _ = select.select(list(socks.values()), [], [], 0.25)
            for s in ready:
                p = s.getsockname()[1]
                try:
                    data, addr = s.recvfrom(65535)
                except OSError:
                    continue
                st = stats[p]
                st['n'] += 1
                st['bytes'] += len(data)
                if st['first'] is None:
                    st['first'] = now
                    st['sample'] = data[:16]
                    seen_any = True
                st['last'] = now

            if now - t_report >= args.report:
                elapsed = now - t_start
                print(f"  t={elapsed:6.1f}s")
                for p in sorted(socks):
                    st = stats[p]
                    if st['n'] == 0:
                        print(f"    {p} ({PORT_NAMES.get(p,'?'):7s})  -- no packets")
                        continue
                    span = max(1e-3, st['last'] - st['first'])
                    kbps = st['bytes'] * 8 / span / 1000.0
                    print(f"    {p} ({PORT_NAMES.get(p,'?'):7s})  "
                          f"{st['n']:7d} pkts  {st['bytes']/1e6:8.2f} MB  "
                          f"{kbps:8.1f} kbit/s")
                print()
                t_report = now
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for s in socks.values():
            s.close()

    print("=== summary ===")
    ok = False
    for p in sorted(socks):
        st = stats[p]
        if st['n'] == 0:
            print(f"  {p} ({PORT_NAMES.get(p,'?'):7s})  nothing received")
            continue
        ok = True
        span = max(1e-3, st['last'] - st['first'])
        print(f"  {p} ({PORT_NAMES.get(p,'?'):7s})  {st['n']} pkts, "
              f"{st['bytes']/1e6:.2f} MB, {st['bytes']*8/span/1000:.1f} kbit/s, "
              f"{describe_rtp(st['sample'])}")
    if not ok:
        print("\nNothing arrived. Check, in order:")
        print("  1. the robot is powered on and the camera nodes are running")
        print("  2. this host has an address in 192.168.123.0/24 (ip -br addr)")
        print("  3. the SENDER's config sets IpLastSegment to this host's last octet")
        print("  4. nothing else already holds the port (SO_REUSEADDR is on, so a")
        print("     second listener silently steals packets)")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
