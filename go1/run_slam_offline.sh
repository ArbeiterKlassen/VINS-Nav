#!/bin/bash
# run_slam_offline.sh — 对回传双目数据跑离线 SLAM（可在录到数据后一键出轨迹/地图）
#
# 用法:
#   source /opt/ros/noetic/setup.bash
#   ./run_slam_offline.sh --bag /tmp/go1_stereo_test.bag --out /tmp/slam_out \
#       [--duration 30] [--no-imu] [--ov-config <openvins estimator_config.yaml>]
#
# 两条路径:
#   1) 默认（无 IMU 也能跑）: rtabmap_odom/stereo_odometry + rtabmap_slam/rtabmap
#      → 输出 $OUT/map.pgm/.yaml（栅格图）、$OUT/rtabmap.db、可选 $OUT/poses/cloud
#   2) 给 --ov-config 且 bag 内有 /go1/imu: OpenVINS run_subscribe_msckf
#      （Go1 的 estimator_config.yaml 尚待生成，见 TASKS.md 第 4 项）
#
# 注意: 会自带 roscore（若本机没有）；退出时清理自己启动的节点。
set -u
BAG=""; OUT=""; DUR=0; OV_CONFIG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --duration) DUR="$2"; shift 2;;
    --ov-config) OV_CONFIG="$2"; shift 2;;
    --no-imu) USE_IMU=no; shift;;
    *) echo "unknown arg: $1"; exit 1;;
  esac
done
[ -z "$BAG" ] && { echo "need --bag"; exit 1; }
[ -z "$OUT" ] && OUT=/tmp/slam_out
mkdir -p "$OUT"
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws_ov/devel/setup.bash" 2>/dev/null || true

ROSCORE_PID=""
if ! pgrep -f "rosmaster --core" >/dev/null 2>&1; then
  echo "[slam] starting roscore"
  roscore >/tmp/roscore_slam.log 2>&1 &
  ROSCORE_PID=$!
  sleep 3
  # 离线回放使用仿真时间：节点时钟跟随 bag 的 /clock，
  # 否则 bag 时间戳相对节点启动时刻是"过去"，TF 缓存会拒绝（extrapolation）。
  rosparam set /use_sim_time true >/dev/null 2>&1 || true
else
  echo "[slam] WARN: 使用已存在的 ROS master；bag 时间戳较旧时可能丢帧（建议先停掉其它 ROS）"
fi
PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  if [ -n "$ROSCORE_PID" ]; then
    kill "$ROSCORE_PID" 2>/dev/null || true
    # rosmaster/rosout 实际是 python3 进程，pkill -x 匹配不到，用 -f
    pkill -f "rosmaster --core" 2>/dev/null || true
    pkill -f "/opt/ros/noetic/bin/rosout" 2>/dev/null || true
    rosparam set /use_sim_time false >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

# 注意: 不按话题过滤回放（rosbag play 的 --topics 参数解析易踩坑；
#       输入 bag 应只含本管线所需话题，见 make_stereo_bag.py）

if [ -n "$OV_CONFIG" ]; then
  echo "[slam] OpenVINS path: config=$OV_CONFIG"
  rosrun ov_msckf run_subscribe_msckf \
    _verbosity:=INFO _config_path:="$OV_CONFIG" \
    _use_stereo:=true _max_cameras:=2 _publish_global_to_imu_tf:=false &
  PIDS+=($!)
  sleep 3
  # bag 需包含 /go1/imu（见 make_stereo_bag.py --imu-csv）
else
  echo "[slam] RTAB-Map stereo path (visual only)"
  rosrun rtabmap_odom stereo_odometry \
    _frame_id:=cam0 _odom_frame_id:=odom _approx_sync:=true \
    _publish_tf:=true _wait_for_transform_duration:=1.0 \
    left/image_rect:=/go1/cam0/image_raw \
    right/image_rect:=/go1/cam1/image_raw \
    left/camera_info:=/go1/cam0/camera_info \
    right/camera_info:=/go1/cam1/camera_info &
  PIDS+=($!)
  sleep 2
  rosrun rtabmap_slam rtabmap --delete_db_on_start \
    _frame_id:=cam0 _odom_frame_id:=odom _subscribe_stereo:=true \
    _subscribe_depth:=false _subscribe_scan:=false _approx_sync:=true \
    _wait_for_transform_duration:=1.0 _gen_scan:=false \
    left/image_rect:=/go1/cam0/image_raw \
    right/image_rect:=/go1/cam1/image_raw \
    left/camera_info:=/go1/cam0/camera_info \
    right/camera_info:=/go1/cam1/camera_info \
    odom:=/odom &
  PIDS+=($!)
  sleep 3
fi

echo "[slam] playing bag: $BAG"
if [ "$DUR" -gt 0 ]; then
  rosbag play -u "$DUR" --clock --quiet "$BAG"
else
  rosbag play --clock --quiet "$BAG"
fi
sleep 5

echo "[slam] saving artifacts to $OUT ..."
timeout 15 rosrun map_server map_saver map:=/rtabmap/grid_map -f "$OUT/map" >/dev/null 2>&1 \
  && echo "  grid map: $OUT/map.pgm + map.yaml" \
  || echo "  (grid map not available — 数据太短/无运动时正常)"
if command -v rtabmap-export >/dev/null 2>&1; then
  # 注意: --output 是文件名（前缀），输出目录用 --output_dir
  timeout 30 rtabmap-export --poses --output poses --output_dir "$OUT" "$HOME/.ros/rtabmap.db" >/dev/null 2>&1 \
    && echo "  poses exported: $OUT/poses_poses.txt"
  timeout 30 rtabmap-export --cloud --output cloud --output_dir "$OUT" "$HOME/.ros/rtabmap.db" >/dev/null 2>&1 \
    && echo "  cloud exported (see $OUT/)"
else
  echo "  (rtabmap-export not found)"
fi
cp "$HOME/.ros/rtabmap.db" "$OUT/rtabmap.db" 2>/dev/null && echo "  database: $OUT/rtabmap.db"
echo "[slam] done"
