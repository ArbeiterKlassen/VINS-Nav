#!/bin/bash
# Offline bag processing: sensor data → pseudo_stereo → OpenVINS → RTAB-Map → map + point cloud
# Usage: ./process_bag.sh [bag_path] [output_dir]
set -e

BAG="${1:-$HOME/house_full.bag}"
OUT="${2:-$HOME/catkin_ws_ov/maps/house_offline}"
CONFIG="$HOME/catkin_ws_ov/src/open_vins/config/rgbd_stereo/estimator_config.yaml"

echo "============================================"
echo " Offline Bag Processing Pipeline"
echo " Bag:   $BAG"
echo " Output: $OUT"
echo "============================================"

# Step 0: Clean up from previous runs
killall -9 rosmaster rosout roscore 2>/dev/null || true
sleep 2
rm -f $HOME/.ros/rtabmap.db

# Step 1: Start roscore
echo "[1/7] Starting roscore..."
roscore &
sleep 3
source /opt/ros/noetic/setup.bash
source $HOME/catkin_ws_ov/devel/setup.bash

# Step 2: Load URDF and start static TF
echo "[2/7] Loading robot description..."
source /opt/ros/noetic/setup.bash
source $HOME/catkin_ws_ov/devel/setup.bash
xacro --inorder $HOME/catkin_ws_ov/src/open_vins/ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.urdf.xacro > /tmp/robot.urdf 2>/dev/null
python3 -c "
import rospy
rospy.init_node('load_urdf', anonymous=True)
with open('/tmp/robot.urdf') as f:
    rospy.set_param('/robot_description', f.read())
print('URDF loaded: %d bytes' % len(open('/tmp/robot.urdf').read()))
"
rosrun robot_state_publisher robot_state_publisher _publish_frequency:=50.0 &
rosrun tf static_transform_publisher 0 0 -0.078 0 0 0 imu base_footprint 100 &
rosrun tf static_transform_publisher 0 0 0 0 0 0 global odom 100 &
rosrun tf static_transform_publisher 0 0 -0.078 0 0 0 odom base_footprint 100 &
sleep 2

# Step 3: Start pseudo-stereo
echo "[3/7] Starting pseudo-stereo..."
rosrun ov_msckf pseudo_stereo.py _baseline:=0.08 &
sleep 2

# Step 4: Start OpenVINS
echo "[4/7] Starting OpenVINS..."
rosrun ov_msckf run_subscribe_msckf \
  _verbosity:=INFO \
  _config_path:="$CONFIG" \
  _use_stereo:=true \
  _max_cameras:=2 \
  _publish_global_to_imu_tf:=false \
  __name:=ov_msckf &
sleep 3

# Step 5: Start odom_tf_pub
echo "[5/7] Starting TF bridges..."
rosrun ov_msckf odom_tf_pub.py &
sleep 1

# Step 6: Start RTAB-Map
echo "[6/7] Starting RTAB-Map..."
rosrun rtabmap_slam rtabmap --delete_db_on_start \
  _frame_id:=base_footprint \
  _odom_frame_id:=global \
  _subscribe_depth:=true \
  _subscribe_scan:=false \
  _approx_sync:=true \
  _wait_for_transform_duration:=0.2 \
  _subscribe_odom_info:=false \
  _gen_depth:=false \
  _gen_scan:=false \
  __ns:=rtabmap \
  rgb/image:=/camera/rgb/image_raw \
  depth/image:=/camera/depth/image_raw \
  rgb/camera_info:=/camera/rgb/camera_info \
  odom:=/ov_msckf/odomimu &
sleep 5

# Wait for OV to subscribe to both cameras
echo "Waiting for OpenVINS to subscribe to camera topics..."
for i in $(seq 1 30); do
  if rosnode info /ov_msckf 2>/dev/null | grep -q "/camera/right/image_raw"; then
    echo "  OV subscribed OK"
    break
  fi
  sleep 1
done

# Step 7: Play the bag
echo "[7/7] Playing bag (this will take ~16 minutes)..."
echo "  Topics: rgb + depth + camera_info + imu + clock"
rosbag play "$BAG" --clock --quiet \
  --topics /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info /imu /clock

# Bag finished
echo "Bag playback complete. Waiting for final processing..."
sleep 5

# Export grid map
echo "Exporting grid map..."
mkdir -p "$(dirname "$OUT")"
for i in $(seq 1 10); do
  if rosrun map_server map_saver map:=/rtabmap/grid_map -f "$OUT" 2>/dev/null; then
    echo "  Grid map saved: $OUT.pgm + $OUT.yaml"
    break
  fi
  sleep 2
done

# Export point cloud from RTAB-Map database
echo "Exporting point cloud..."
if rtabmap-export --cloud --output "$OUT" "$HOME/.ros/rtabmap.db" 2>/dev/null; then
  echo "  Point cloud saved: $OUT.ply (or .ply in output dir)"
else
  echo "  Point cloud export failed (may need rtabmap-export from rtabmap package)"
fi

echo "============================================"
echo " Pipeline complete"
echo " Map:  $OUT.pgm + $OUT.yaml"
echo " Cloud: $OUT.ply (or check output dir)"
echo "============================================"
