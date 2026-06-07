# OpenVINS Pseudo-Stereo VIO + 离线建图技术栈

## 最终方案：直接 bag 处理

绕过 RTAB-Map，直接从 bag 读取 OV 位姿和深度图拼接点云。

```
bag → /tf (global→imu) → 机器人位姿
bag → /camera/depth/image_raw → 深度图  
bag → /camera/rgb/camera_info → 内参

逐帧: 位姿 × 深度 × 内参 → 全局点云 → 2D 占用网格
```

结果：462×292 (23m×15m)，27M 点，Z 范围 -0.13~2.50m，6216 障碍物。

```bash
python3 build_map.py ~/house_full.bak ~/maps/output
python3 postprocess_map.py ~/maps/output.yaml ~/maps/output_final.pgm
```

## RTAB-Map 离线模式的教训

1. **odom 话题 remap 在 rosrun/roslaunch 中均不产生订阅**——RTAB-Map 的 odom 订阅是延迟创建且依赖 camera sync 先成功。用 topic_tools relay 到 /rtabmap/odom 也无订阅者。

2. **odom_sensor_sync 的选择**——false 时从 TF 读里程计（不订阅话题），true 时等 4 话题同步但 odom 话题仍不出现。两者都无法在离线模式下获取 odometry。

3. **图优化坍塌**——即使 Total odometry length=82m、660 poses、1135 links，优化后产生 "0 poses"，所有点云被压成平面。根因是视觉词典不完整（DB 多次重建/重启导致）和 odometry 约束不足。

4. **/tf_static 多 publisher 覆盖**——多个 tf2_ros StaticTransformBroadcaster 写同一 latched topic，后来者覆盖前者。解决方案：所有静态 TF 由一个 publisher 发布。

5. **/tf 用 ros::Time::now() 产生 Unix 时间戳**——与 bag 的 sim time 不匹配。必须用 tf2_ros 发布 timeless 变换，或在 bag 播前设 use_sim_time。

6. **use_sim_time 必须最优先设置**——在所有 ROS 节点启动之前，否则节点使用 Wall clock 而非 Sim clock。

## 在线方案（Gazebo 实时）

TF 链: `map(RTAB-Map) → global(dyn VIO) → odom(dyn TF) → base_footprint(static TF)`

关键：
- dyn_odom_tf.py 从 /ov_msckf/odomimu 读取位姿，动态发布 global→odom
- tf static_transform_publisher 发布 odom→base_footprint (0,0,-0.078) 到 /tf
- OV publish_global_to_imu_tf=false 避免 333Hz TF 洪水
- base_footprint 只有一个父帧（odom），不能同时有 imu→base_footprint

## 配置

estimator_config.yaml: try_zupt=true, init_imu_thresh=0.05, init_max_features=200, num_pts=600, MLE=0
kalibr_imu_chain.yaml: noise_density=0.00083 (Allan 实测), random_walk=1e-6/1e-7
kalibr_imucam_chain.yaml: baseline=0.08m, cam0=/camera/rgb, cam1=/camera/right

## 性能

尺度 0.991x，3.1m 漂移 2.8cm (0.9%)，方位误差 0.0°，初始化 <1ms，特征 190-276。
