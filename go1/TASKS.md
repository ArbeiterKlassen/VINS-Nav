# Go1 相机 / VIO / SLAM 任务清单（2026-10-17）

状态：✅ 完成 / 🔄 进行中 / ⏳ 待办 / 🧊 等机器人上电

## 已完成
- ✅ 单目 H.264 推流/接收/解码（端口 9001，44 帧 PNG 已视觉核实）——见 `FINDINGS-2026-10-11.md`
- ✅ 8007 低层状态解码（`lowstate_rx.py`，500 Hz 实测 PASS）
- ✅ 8082 单客户端判读纠正 + `/high_state` 采样方法
- ✅ mode-0 残留事件闭环（断电重启恢复）+ 常态化 SOP
- ✅ **双目适配关键结论**：SDK 的 UDP H.264 推流只发单目（Transmode 0/1/3 实测均单目
  928×800，板载头文件无 Transmode）；本地 API `getStereoFrame` / `getRectStereoFrame`
  可取左右目（各 928×800、带 µs 时间戳、有真实视差）；**校正输出为 180° 倒置**。
- ✅ 双目 TCP+JPEG 桥：发送端 `stage/stereo_tcp_sender.cc`（.13 已编译通过）、
  接收端 `go1_stereo_receiver.py`（VM 就绪：可选 ROS 图像/camera_info、存 PNG）
- ✅ 空间标定方案调研（Kalibr pinhole-equi / OpenCV fisheye / Mei / SDK 校正对比）

## 待办（按优先级）
1. 🧊 **双目桥端到端实测**（上电后）：重新部署 `.13:/dev/shm`（断电已清空）→
   `bench_rect` / `bench_raw` 实测帧率（只 JPEG）→ 接收端收流验证（左右目一致性、
   时延、丢帧）→ 定工作模式（分辨率/帧率/质量）。
2. 🧊 **录制一段双目视频**：**运动来源已定 = 遥控手柄**（狗先用手柄站起/行走；
   录制期间**不得启动任何 8082 客户端**，sport 客户端语义留给手柄）。
   工具：`go1/record_stereo_imu.sh`（双目桥 + 8007 IMU → rosbag，IMU 已获用户确认）。
3. ✅/🔄 **离线 SLAM 回放**：`go1/run_slam_offline.sh` + `go1/make_stereo_bag.py`
   （PNG 对 [+IMU CSV] → rosbag；无 IMU 走 RTAB-Map stereo，有 IMU 走 OpenVINS）。
   **冒烟已通过**（节点订阅 / 仿真时间 / 清理均正确；静态数据无运动）。
   **建图核心已用真实运动数据复跑验证**：`house_full.bag` [300s,420s] 段
   （3600 个 VIO 位姿 + depth）→ 312 万点 → 251×274 占用栅格图
   （`/tmp/map_demo/house_seg.png`，演示脚本 `/tmp/build_map_demo.py`）。
   待 Go1 真实录制数据后按同一路径（`run_slam_offline.sh`）验证精度。
4. 🔄 **OpenVINS 适配**：生成 Go1 双目配置（cam0/cam1 内参 + `T_imu_cam` + IMU 噪声）。
   **IMU 已获用户确认启用**（8007 轮询，只发非法填充触发包）——
   `go1/go1_imu_publisher.py` 提供 `/go1/imu`；`record_stereo_imu.sh` 一并录制。
5. 🔄 **空间标定落地**：先用 SDK 校正图 + 估计内参（hFov 90° ⇒ fx≈464@928 宽，
   基线用出厂 Translation≈25.09 mm）冒烟；再用 OpenCV fisheye 棋盘格实测标定
   （或 Kalibr）替换。`go1/stereo_depth_demo.py` 已能出视差/深度图与距离估计
   （当前为估计内参，标定后可提精度）。
6. ✅ **站立脚本**：`go1/go1_stand_up.py`（mode 6/8，auto 6→8 回退；CRC 已用官方
   amd64 静态库三组向量**逐字节校验**；含 HighState 回读 mode/bodyHeight）。
   **尚未实机执行**（等狗上电 + 用户按 SOP 确认）。
7. ⏳ 相机节点状态管理：`.13` 自启节点断电后自动恢复；需要时再按 `camera_nodes.sh`
   record→kill（流程见 `README.md` §6）。

## 停机原因（2026-10-17）
- 机器人电池耗尽（`.13` 断电不可达）。等待充电/上电后从第 1 项继续；
  断电会清空 `.13` 的 `/dev/shm`（编译产物需重新部署，源码都在 `stage/`）。
