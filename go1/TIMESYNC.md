# Go1 time base — design options

Design only; nothing here has been run against the robot. Written 2026-10-11 while
the robot was charging.

---

## 1. What the problem actually is

Two clocks produce the two streams a VIO backend needs, and they have never been
introduced:

| Stream | Produced by | Stamped by |
|---|---|---|
| IMU (500 Hz, via `unitree_legged_sdk`) | the VM reading UDP | VM clock |
| Camera (H.264 over RTP) | the Jetson | Jetson clock, *or* the VM on arrival |

The Jetsons have **no RTC** — `/dev/rtc` is absent, so on every boot their clock
jumps to whatever the filesystem image was built with. The field notes in
`../docs/unitree_go1.md` §8.3 recorded `2018-01-29` on `.13`. The RTP/H.264 stream
also carries no wall-clock field we control; RTP has a 32-bit media-clock timestamp,
but mapping that to a wall clock still needs a reference point.

**Why it matters for VIO, and how much.** OpenVINS *estimates* a constant
camera-to-IMU time offset (`calib_cam_timeoffset`), so a fixed offset is not a
problem — it gets absorbed. Two things are problems:

- **Jitter**: a variable offset is noise the estimator cannot remove, and it leaks
  into velocity and scale.
- **Discontinuities**: a clock that steps mid-run (an NTP correction, a manual
  `date -s`) produces a timestamp jump that looks like a violent motion.

So the design goal is **stability, not absolute accuracy**. A constant 200 ms offset
is harmless; a 2 ms offset that wanders is not.

Sanity check on the magnitude: at the Go1's walking speed (~0.5 m/s), 1 ms of
timestamp error is 0.5 mm of apparent motion. Even 10 ms is fine *provided it is
constant*.

---

## 2. The key simplification

**If both streams are stamped on the VM clock, no clock synchronisation is needed
at all.**

The camera arrives at the VM; the IMU arrives at the VM. Timestamp both at arrival
and they share one clock by construction. The residual is the camera pipeline
latency (capture → encode → network → decode), which is:

- **constant** for a fixed pipeline, and therefore absorbed by `calib_cam_timeoffset`
- **small** on a wired LAN through a bridged NIC

This works with no RTC, no NTP, no SSH, and nothing installed on the robot. It is
the baseline, and it may well be sufficient on its own.

What it does *not* give you: the ability to correlate a frame with anything logged
on the Jetson, and no visibility into how much of the latency is network versus
encode versus decode. Those are diagnostic conveniences, not VIO requirements.

---

## 3. Routes, with trade-offs

### Route A — arrival-time stamping (no sync) — **baseline, recommended first**

Stamp every camera frame with the VM's clock the moment it is decoded; the IMU is
already stamped that way.

| | |
|---|---|
| Effort | none — it is the default if we do nothing |
| Works without RTC | yes |
| Touches the robot | no |
| Risk | decoder frame reordering (B-frames) makes latency vary; wired-LAN jitter adds ~0.1–1 ms |
| Gives up | no cross-machine log correlation |

**Mitigation for the reordering risk:** confirm the Jetson's `omxh264enc` is producing
a stream without B-frames (baseline profile, no frame reordering), or decode with a
pipeline that reorders before we stamp. Worth checking once we see a stream.

### Route B — one-shot clock step over SSH — **cheap, do it, but not for VIO**

```bash
VM_EPOCH=$(date +%s.%N)
ssh unitree@192.168.123.13 "echo 123 | sudo -S date -s '@${VM_EPOCH}'"
```

| | |
|---|---|
| Effort | one command, ~1 s |
| Works without RTC | until the next boot — must be repeated every session |
| Touches the robot | sets the clock; nothing installed, no files changed |
| Risk | a **step**, so any process on the Jetson holding a monotonic-to-wall mapping sees a jump |
| Gives up | nothing; it is additive |

Do this at the start of a session, **before** starting any capture, so nothing is
running when the clock jumps. It makes Jetson logs readable and lets us measure the
true capture-to-arrival latency by comparing the Jetson's own timestamps with the
VM's arrival times.

### Route C — chrony, VM as server — **only if it becomes necessary**

The VM would serve NTP on `192.168.123.0/24`; each Jetson would run chronyd against
it, giving continuous slewing instead of a step.

| | |
|---|---|
| Effort | high: `chrony` is **not installed on the VM** (verified) and installing needs sudo apt; the Jetsons additionally need it installed **and they likely have no internet to apt-install from** |
| Works without RTC | yes, and it re-holds drift automatically |
| Touches the robot | installs a package and writes config — a real change to the robot |
| Risk | the Jetsons' clocks start **years** off, so chrony must be told to `makestep` on first contact or it will try to slew a decade |

The install path on the Jetsons is the likely blocker. Worth revisiting only if
Route B turns out to be too manual.

### Route D — custom NTP-style sync over SSH

Our own script: measure the offset with NTP's four-timestamp exchange over SSH, then
either step or slew. No installs, works with what is already there.

| | |
|---|---|
| Effort | medium — we write and maintain it |
| Works without RTC | yes, per session |
| Touches the robot | clock only |
| Risk | SSH round-trip jitter enters the estimate; the offset can be measured but not held without a daemon |

A minimal version is ~30 lines and gives a *measured* offset with an uncertainty,
which is more honest than Route B's blind step. Reasonable middle ground if Route B's
step proves disruptive.

---

## 4. Recommendation

1. **Start with Route A.** Do not synchronise anything for the first VIO attempt.
   Both streams on the VM clock is the simplest correct thing, and it needs nothing
   from the robot.
2. **Add Route B per session**, before capture starts, purely for diagnosability —
   it lets us measure the camera pipeline latency instead of guessing.
3. **Measure before escalating.** If the VIO's estimated `calib_cam_timeoffset` is
   stable, Routes C and D are unnecessary. If it wanders, the wander tells us which
   layer is at fault, and *then* pick a fix.
4. **Do not add chrony pre-emptively.** It is the most work, touches the robot most,
   and solves a drift problem we have not yet shown we have.

---

## 5. Verification

### 5.1 Offset and jitter between VM and Jetson (Route B/D)

Run from the VM, 20 samples, and look at the *spread* rather than the mean:

```bash
for i in $(seq 20); do
  t1=$(date +%s.%N)
  t2=$(ssh -o BatchMode=yes unitree@192.168.123.13 'date +%s.%N')
  t3=$(date +%s.%N)
  python3 -c "
t1,t2,t3=$t1,$t2,$t3
mid=(t1+t3)/2
print(f'{t2-mid:+.6f}')"
done | sort -n
```

The output is `jetson_clock − vm_clock`, each sample including half the SSH round
trip as error. Interpretation:

- **Median** is the offset — after Route B this should be near zero
- **Spread (max−min)** is the achievable accuracy over SSH; on a wired LAN expect
  a few ms. If it is tens of ms, SSH is not a good sync channel and Route C/D
  deserve another look
- **Stability over time** — repeat after 10 minutes. A drifting offset means the
  Jetsons' clocks run at a genuinely different rate (possible with a bad crystal),
  and no one-shot step can fix that

### 5.2 Camera pipeline latency (needs a stream)

With Route B applied, compare a frame's Jetson-side capture stamp against its VM
arrival stamp. Since the H.264 stream may not carry a usable capture stamp, the
practical version is to compare the Jetson's `date` at the moment capture starts
with the VM's arrival of the first frame — a coarse but sufficient estimate.

What we want: latency **constant**, not latency small. Plot arrival interval for a
few hundred frames; a bimodal distribution means frame reordering.

### 5.3 The one that actually matters

Watch `calib_cam_timeoffset` in the VIO output over a run:

- **Converges and holds** → the time base is good enough, stop here
- **Wanders slowly** → the clocks are drifting relative to each other; Route C or D
- **Jumps** → something stepped a clock mid-run (an NTP correction, or Route B run
  late); fix the sequencing, not the sync method

---

## 6. What is needed from the robot side

Once the robot is powered, before any VIO work:

1. `date -Is` and `ls /dev/rtc*` on each of `.13/.14/.15` — confirm the no-RTC claim
   and record how far off each clock is
2. `chronyd --version` / `which ntpd` — is any time daemon already present?
3. Whether the Jetsons can reach the internet at all (`ping 8.8.8.8` from `.13`) —
   this decides whether Route C is even possible
4. The `omxh264enc` profile actually in use — settles the frame-reordering question
   in §5.2

Items 1–3 are read-only. Item 4 is read-only too.
