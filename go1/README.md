# Go1 bring-up

Working notes for connecting the Unitree Go1 to the VINS-Nav stack. Hardware
background is in [`../docs/unitree_go1.md`](../docs/unitree_go1.md); this file is
the operational side.

**Status 2026-10-11:** network configured, camera SDK builds and runs, end-to-end
unverified — the robot's battery is flat and it is powered off. Everything below
marked *verified* was checked without the robot; everything marked *to confirm*
needs it powered on.

---

## 1. Network

The VM reached `192.168.123.x` through the host's NAT even before any changes, but
NAT only works for flows the VM *initiates*. The camera streams are **pushed by the
Jetson to a fixed address**, so the VM needs a real address on that subnet.

A second NIC was added for this:

| Interface | Address | Role |
|-----------|---------|------|
| `ens33` | `192.168.38.134/24` | VMnet8 NAT — internet, unchanged |
| `ens37` | **`192.168.123.101/24`** | bridged to the host's 2.5G port — robot subnet |

*Verified:* `ens37` is up, carrier present, and the route is now direct rather than
through the NAT gateway:

```
$ ip route get 192.168.123.10
192.168.123.10 dev ens37 src 192.168.123.101
```

**`IpLastSegment` must be set to 101** in the sender's config before any streaming
(see §3). The receiver's address is `192.168.123.<IpLastSegment>`.

---

## 2. Two different streaming mechanisms — do not mix them up

There are **two unrelated** UDP schemes in the camera SDK, and the port numbers
differ. Reading only one source gives the wrong answer, which happened during the
first pass of this research.

| | `StereoCameraCommon::startCapture(udpFlag, …)` | `example_putImagetrans` / `example_getimagetrans` |
|---|---|---|
| Ports | `9000 + camera position number` | **`9201`–`9205`** |
| Comment in source | `StereoCameraCommon.hpp:594` | `example_getimagetrans.cc` header |
| Sender gstreamer | `… ! rtph264pay ! udpsink host=192.168.123.SEG port=hostPort` | internal, not shown |
| Receiver pipeline | — | `udpsrc … ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! appsink` |

The image-transmission path is the documented, example-backed workflow, so **use
9201–9205**:

| Port | Camera |
|------|--------|
| 9201 | front |
| 9202 | chin |
| 9203 | left |
| 9204 | right |
| 9205 | abdomen |

---

## 3. Who runs what

`UnitreeCamera cam("…yaml")` opens a **local V4L2 device**. It therefore only works
on the machine physically attached to that camera — one of the three Jetsons, not
the VM. The split is:

```
Jetson (.13 / .14 / .15)                    VM (192.168.123.101)
  example_putImagetrans                       udp_probe.py  (connectivity)
    opens /dev/video*, H.264 encodes            or gst-launch (decode)
    pushes RTP/UDP to 192.168.123.101   ───►
    ports 9201–9205
```

### Config files (in the SDK root)

| File | `Transmode` | `FrameSize` | Use |
|------|-------------|-------------|-----|
| `stereo_camera_config.yaml` | `-1` (no streaming) | 928×400 | calibration readout only |
| `trans_rect_config.yaml` | `0` (right, raw) | 1856×800 | full stereo transmission |

Both currently have `IpLastSegment: 15`. **Change it to `101`** for the VM.

`Transmode` values: `0` right raw, `1` stereo raw, `2` right rectified (FOV
adjustable 60–140°), `3` stereo rectified, `-1` do not transmit.

> **Gotcha from the source:** for stereo modes, *the left and right outputs are
> swapped* — left lands in the right image and vice versa (`example_putImagetrans.cc`).
> Worth re-checking empirically before trusting either channel.

### Freeing the camera first

The Jetsons autostart camera nodes that hold `/dev/video*`, so the SDK cannot open
the device until they are stopped. The source gives the commands:

```bash
ps -A | grep point | awk '{print $1}' | xargs kill -9
ps -aux | grep mqttControlNode  | grep -v grep | head -n 1 | awk '{print $2}' | xargs kill -9
ps -aux | grep live_human_pose  | grep -v grep | head -n 1 | awk '{print $2}' | xargs kill -9
```

or stop `~/Unitree/autostart/02camerarosnode` (frees the camera) and
`~/Unitree/autostart/04imageai` (frees compute) properly.

**This disables the robot's own person-tracking / image AI.** That is a real
functional loss and should be agreed before doing it, not discovered afterwards.

---

## 4. Building the SDK on this VM — resolved

The SDK ships **prebuilt static libraries** for `arm64` and `amd64`, so only the
examples need building. The reported trap is that it is pinned to OpenCV 4.1.1 and
segfaults with newer versions; this VM has **4.2.0**.

*Verified:* it builds and runs.

```
examples built:  example_getCalibParamsFile   example_getRawFrame
                 example_getDepthFrame        example_getRectFrame
                 example_getPointCloud        example_putImagetrans
                 example_getimagetrans
```

The lib's undefined symbols are ordinary core/imgproc/calib3d calls (`cv::Mat`,
`cv::cvtColor`, `cv::remap`, `cv::StereoBM`, …), all present in 4.2. Running
`example_getCalibParamsFile` with no camera attached exits **0** with a clean
"No camera" message — no segfault.

One real build blocker, unrelated to OpenCV: **`libudev-dev` is missing** (the
runtime `libudev1` is present, but not the `.so` symlink). Worked around without
touching the system by pointing the linker at a stub directory:

```bash
mkdir -p /tmp/ucsdk_build/stubs
ln -sf /lib/x86_64-linux-gnu/libudev.so.1 /tmp/ucsdk_build/stubs/libudev.so
cd /tmp/ucsdk_build
cmake ~/go1_ws/src/UnitreecameraSDK -DCMAKE_EXE_LINKER_FLAGS="-L/tmp/ucsdk_build/stubs"
make -j4
```

Binaries land in `~/go1_ws/src/UnitreecameraSDK/bins/`. For a permanent setup
install `libudev-dev` properly instead.

### Decoding on the VM

The SDK's receive pipeline uses **`omxh264dec`** — the Jetson hardware decoder,
which does not exist on x86. Software equivalents `h264parse` and `avdec_h264` are
also missing here (they live in `gstreamer1.0-plugins-bad` and
`gstreamer1.0-libav`). Present: `udpsrc`, `rtph264depay`, `videoconvert`,
`appsink`, `autovideosink`.

That does not block the first test — **proving the packets arrive needs no
decoder**. Once the plugins are installed, the VM-side pipeline is the SDK's with
one substitution:

```bash
# Jetson / the SDK's own example
gst-launch-1.0 udpsrc address=192.168.123.101 port=9201 \
  ! application/x-rtp,media=video,encoding-name=H264 \
  ! rtph264depay ! h264parse ! omxh264dec ! videoconvert ! autovideosink

# x86 VM: swap the hardware decoder for the software one
gst-launch-1.0 udpsrc address=192.168.123.101 port=9201 \
  ! application/x-rtp,media=video,encoding-name=H264 \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

---

## 5. Test procedure once the robot has power

Ordered cheapest-first; each step is meaningful on its own.

1. **Connectivity** — from the VM:
   ```bash
   ping -c3 192.168.123.10 192.168.123.13 192.168.123.161
   ip route get 192.168.123.10        # expect dev ens37, not the NAT gateway
   ```
2. **Reverse direction** — from the robot side, ping `192.168.123.101`. The camera
   path depends on the Jetson reaching the VM, which the NAT path never exercised.
3. **Calibration readout** — SSH to the Jetson owning the camera of interest, stop
   the autostart camera nodes, then run `bins/example_getCalibParamsFile`. Confirm
   `output_camCalibParams.yaml` appears and holds a sane baseline. `Transmode: -1`
   in `stereo_camera_config.yaml` means this does *not* stream, so it is safe.
4. **Stream receives** — set `IpLastSegment: 101`, run `example_putImagetrans` on
   the Jetson, and on the VM:
   ```bash
   python3 go1/udp_probe.py --ports 9201 9202 --seconds 30
   ```
   Success is packets arriving with an `RTP v2` header. Decoding is step 5's problem.
5. **Decode** — install `gstreamer1.0-plugins-bad` and `gstreamer1.0-libav`, then
   run the `avdec_h264` pipeline above and look at the picture.

Nothing in this list sends a control packet. Control comes later, and separately:
idle/stand first, robot on the ground, someone watching.

---

## 6. Open questions

- Which Jetson owns which camera, and whether the head pair is the best VIO choice
  (widest baseline? best forward view?). The board/camera mapping in
  `docs/unitree_go1.md` came from documentation, not from the unit.
- Whether `h264parse`/`avdec_h264` decode at 30 fps 1856×800 on this VM, or whether
  the stream has to be reduced via `FrameSize` / `Transrate` on the sender.
- End-to-end latency and whether the RTP stream carries usable timestamps for VIO —
  the stream is H.264 over RTP with no obvious time base, and the Jetsons have no
  RTC, so timestamping needs thought before this can feed an estimator.
