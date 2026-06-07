OpenVINS Pseudo-Stereo VIO + 离线建图系统

环境: ROS Noetic, ~/catkin_ws_ov/

================================================================================
最终方案: build_map.py — 直接读 bag，绕过 RTAB-Map
================================================================================

bag → /tf (global→imu) → 机器人位姿
bag → /camera/depth/image_raw → 深度图
bag → /camera/rgb/camera_info → 内参

逐帧: OV 位姿 × 深度图 × 内参 → 全局点云 → 2D 占用网格

结果: 462×292 (23m×15m), 27M 点, Z: -0.13~2.50m, 6216 障碍物

脚本: scripts/build_map.py
用法: python3 build_map.py <bag路径> <输出前缀>

================================================================================
RTAB-Map 离线模式的致命缺陷
================================================================================

1. odom remap 在 rosrun/roslaunch 中均不生效 → RTAB-Map 从不订阅 odom 话题
   改用 topic_tools relay 也无效——RTAB-Map 的 odom 订阅是延迟创建且依赖 camera sync
2. odom_sensor_sync=false → RTAB-Map 从 TF 读里程计，不订阅话题
   odom_sensor_sync=true → RTAB-Map 等 4 话题同步，odom 话题仍不出现
3. 图优化产生 "Optimized graph: 0 poses" → 全部位姿坍塌到平面
   即使 Total odometry length=82m，优化后位姿为 0
4. DB 词典不完整 → "Not found word X (dict size=N)" → 回环检测失效

结论: RTAB-Map 不适合离线 bag 回放场景。在线实时跑没问题，离线必翻车。

================================================================================
已修复的 Bug 总表
================================================================================

| Bug | 根因 | 修复 |
|-----|------|------|
| DB 不变 | use_sim_time 未设置 | rosparam set /use_sim_time true 在所有节点之前 |
| TF Unix/Sim 时间不匹配 | tf static_transform_publisher 用 ros::Time::now() | 改用 tf2_ros StaticTransformBroadcaster |
| /tf_static 被覆盖 | 多个 publisher 写同一 latched topic | 只留一个 publisher，其他发到 /tf |
| global→odom 写死 identity | 静态 TF 覆盖动态里程计 | dyn_odom_tf.py 从 VIO 动态发布 |
| odom→base_footprint 缺失 | TF 链断开 | 单独 static_transform_publisher |
| base_footprint 双父帧 | imu→base 和 odom→base 同时存在 | 删除 imu→base，只保留 odom→base |
| OV TF 333Hz 洪水 | publish_global_to_imu_tf=true | 设为 false |
| odom remap 不生效 | RTAB-Map 延迟创建 odom 订阅 | 放弃 RTAB-Map，改用直接法 |
| 点云 Z 全 0 | RTAB-Map 图优化坍塌 | 直接读 bag TF + depth 拼点云 |
| TF frame 名错误 | bag 中是 global→imu 非 global→odom | 确认 bag 实际 frame 名再读 |

================================================================================
可用的离线建图工作流
================================================================================

# 1. 录制 bag（Gazebo 在线）:
rosbag record -O house.bag \
  /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info \
  /imu /clock /tf /tf_static

# 2. 直接处理:
python3 ~/catkin_ws_ov/scripts/build_map.py ~/house.bak ~/maps/output

# 3. 后处理:
python3 ~/catkin_ws_ov/scripts/postprocess_map.py ~/maps/output.yaml ~/maps/output_final.pgm

================================================================================
在线方案（Gazebo 实时建图）
================================================================================

启动: roslaunch ov_msckf turtlebot3_house_stereo.launch
需要: dyn_odom_tf.py 发布 global→odom 动态 TF
      tf2_ros StaticTransformBroadcaster 发布 odom→base_footprint
      OV publish_global_to_imu_tf=false

TF 链: map(RTAB-Map) → global(dyn VIO) → odom(dyn) → base_footprint(static)

================================================================================
关键文件
================================================================================

config/rgbd_stereo/:
  estimator_config.yaml — ZUPT init, init_imu_thresh=0.05, MLE=0
  kalibr_imu_chain.yaml — noise_density=0.00083 (Allan 实测)
  kalibr_imucam_chain.yaml — baseline=0.08m

scripts/:
  build_map.py           — 离线直接建图（推荐）
  pseudo_stereo.py       — 深度→右目 warp
  dyn_odom_tf.py         — VIO 位姿→动态 TF
  postprocess_map.py     — 地图后处理
  offline_final.sh       — 离线全流程（含 RTAB-Map，不推荐）
  analyze_imu_noise.py   — Allan 方差分析

launch/:
  turtlebot3_house_stereo.launch — 在线全系统
  rtabmap_db.launch      — RTAB-Map 节点

maps/:
  house_direct2.pgm + .yaml      — 离线直出 462×292
  house_direct2_final.pgm + .yaml — 后处理版

docs/:
  openvins.md — 论文格式技术报告
  stack.md   — 技术栈总结
