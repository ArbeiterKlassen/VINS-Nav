# go1/stage — 相机推流最小运行集（2026-10-11 现场）

本目录只保留**我们自己的**配置与操作说明。推流器源码来自 UnitreeCameraSDK 的
示例（版权归 Unitree，不随本仓库分发）；本机与 `.13` 上保留本地副本，Git 忽略
（见仓库根 `.gitignore`）。

## 文件

| 文件 | 用途 |
|---|---|
| `stereo_camera_config.yaml` | 标定读取用：`Transmode: -1`（不推流）、`DeviceNode: 1`（= face 相机 `/dev/video1`）、`IpLastSegment: 101`（VM） |
| `trans_rect_config.yaml` | 推流用：`Transmode: 0`、face 相机、目标 `192.168.123.101`、`FrameSize 1856×800` |
| `example_putImagetrans*.cc` | **不入库**（第三方示例源码，本地副本） |

## 复现推流器（在 `.13` 上，全 `/dev/shm`，零磁盘写入）

1. 从板载 SDK 复制示例到运行目录：
   `cp <SDK>/examples/example_putImagetrans.cc /dev/shm/ucsdk_run/`
2. 两处补丁（板载 SDK 头文件比示例源码新）：
   - `getRectStereoFrame(left,right)` → `getRectStereoFrame(left,right,feim)`（3 参）
   - `cv::waitKey(10)` → `usleep(10000)`（无头运行）
   - 本机已有打好补丁的副本 `example_putImagetrans_headless.cc`（同样不入库）
3. 编译（板载 SDK 头/静态库 + `/usr/local` OpenCV 4.1.1）：
   ```bash
   g++ -std=c++11 -pthread -O2 -DOPENCV_VERSION_4 \
     -I/home/unitree/Unitree/sdk/UnitreeCameraSdk/include \
     -I/usr/local/include/opencv4 \
     example_putImagetrans_headless.cc \
     -L/home/unitree/Unitree/sdk/UnitreeCameraSdk/lib/arm64 \
     -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
     -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev \
     -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio \
     -lopencv_calib3d -lopencv_highgui -lopencv_features2d -lopencv_flann -lopencv_photo \
     -o example_putImagetrans
   ```
4. 运行 / 停止 / 接收解码：见 `../FINDINGS-2026-10-11.md` §2、§4。
   **端口 = 9000 + 相机位置号**（face → **9001**，不是 9201–9205）。

> 证据帧（44×PNG，928×800）未入库（约 38MB），保存在现场 VM 的
> `go1/cam_evidence/`；主机侧亦已取回副本。
