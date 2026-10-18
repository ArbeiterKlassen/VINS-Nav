#!/bin/bash
# record_stereo_imu.sh — 录制「双目 + IMU」rosbag（手柄操控狗移动时使用）
#
# 组成：.13 双目 TCP/JPEG 发送端 → VM 接收端（ROS 图像）
#       VM 8007 IMU 发布器 → /go1/imu
#       rosbag record 以上话题
#
# ⚠️ 录制约束（重要）：
#   * 运动来源 = 遥控手柄（狗需先用手柄站起/行走）；
#   * **不得启动任何 8082 客户端**（ros_udp 等），sport 的客户端语义留给手柄；
#   * 8007 轮询（IMU）已获用户确认，它只发非法填充触发包，不改控制状态；
#   * 狗在开阔区域，有人看护，随时可断电。
#
# 用法:
#   source /opt/ros/noetic/setup.bash
#   ./record_stereo_imu.sh --out /tmp/go1_rec.bag [--seconds 90] [--no-remote]
#   不给 --seconds 时按 Ctrl-C 结束。
set -u
OUT=""; SECS=0; REMOTE=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2;;
    --seconds) SECS="$2"; shift 2;;
    --no-remote) REMOTE=0; shift;;
    *) echo "unknown arg: $1"; exit 1;;
  esac
done
[ -z "$OUT" ] && { echo "need --out"; exit 1; }
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws_ov/devel/setup.bash" 2>/dev/null || true
cd "$(dirname "$0")"

ROSCORE_PID=""
if ! pgrep -f "rosmaster --core" >/dev/null 2>&1; then
  echo "[rec] starting roscore"
  roscore >/tmp/roscore_rec.log 2>&1 &
  ROSCORE_PID=$!
  sleep 3
fi
if rosparam get /use_sim_time 2>/dev/null | grep -q true; then
  echo "[rec] WARN: /use_sim_time=true（离线回放残留？）— 录制需要墙钟，已重置为 false"
  rosparam set /use_sim_time false >/dev/null 2>&1 || true
fi

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
  if [ "$REMOTE" = 1 ]; then
    ssh -o BatchMode=yes -o ConnectTimeout=3 unitree@192.168.123.13 \
      'kill $(cat /dev/shm/ucsdk_run/sender.pid) 2>/dev/null; true' 2>/dev/null || true
  fi
  if [ -n "$ROSCORE_PID" ]; then
    kill "$ROSCORE_PID" 2>/dev/null || true
    pkill -f "rosmaster --core" 2>/dev/null || true
    pkill -f "/opt/ros/noetic/bin/rosout" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [ "$REMOTE" = 1 ]; then
  echo "[rec] starting stereo sender on .13 (TCP 9101)"
  ssh -o BatchMode=yes -o ConnectTimeout=4 unitree@192.168.123.13 \
    'cd /dev/shm/ucsdk_run && (setsid nohup ./stereo_tcp_sender rect 192.168.123.101 9101 > sender.log 2>&1 < /dev/null & echo $! > sender.pid); sleep 1; ps -eo pid,args | awk "\$2==\"./stereo_tcp_sender\""' \
    2>&1 | tail -3
  sleep 1
fi

echo "[rec] starting stereo receiver (ROS)"
python3 go1_stereo_receiver.py --ros > /tmp/stereo_receiver.log 2>&1 &
PIDS+=($!)
sleep 2
echo "[rec] starting IMU publisher (8007, 500Hz)"
python3 go1_imu_publisher.py --hz 500 --csv "${OUT}.imu.csv" > /tmp/imu_pub.log 2>&1 &
PIDS+=($!)
sleep 3

ready=0
for i in $(seq 1 15); do
  if rostopic list 2>/dev/null | grep -q "^/go1/imu$" && rostopic list 2>/dev/null | grep -q "^/go1/cam1/image_raw$"; then
    ready=1; break
  fi
  sleep 1
done
if [ "$ready" != 1 ]; then
  echo "[rec] ERROR: topics not ready — check /tmp/stereo_receiver.log and /tmp/imu_pub.log"
  tail -5 /tmp/stereo_receiver.log /tmp/imu_pub.log 2>/dev/null
  exit 1
fi

echo "[rec] recording -> $OUT"
rosbag record -O "$OUT" \
  /go1/cam0/image_raw /go1/cam1/image_raw \
  /go1/cam0/camera_info /go1/cam1/camera_info /go1/imu > /tmp/rosbag_record.log 2>&1 &
REC_PID=$!
PIDS+=($REC_PID)
if [ "$SECS" -gt 0 ]; then
  echo "[rec] recording ${SECS}s — 现在用手柄操控狗移动"
  sleep "$SECS"
else
  echo "[rec] recording until Ctrl-C — 现在用手柄操控狗移动"
  wait "$REC_PID"
fi
kill "$REC_PID" 2>/dev/null || true
sleep 2
echo "[rec] done"
rosbag info "$OUT" 2>/dev/null | head -14
echo "[rec] next: ./run_slam_offline.sh --bag $OUT --out <dir>"
