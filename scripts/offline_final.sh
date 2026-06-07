#!/bin/bash
# Final offline pipeline — all bugs fixed.
# Usage: ./offline_final.sh [bag_path]
set -e
BAG="${1:-$HOME/house_full.bak}"
OUT="$HOME/catkin_ws_ov/maps/house_offline"
D=$(dirname "$0")
source /opt/ros/noetic/setup.bash
source $HOME/catkin_ws_ov/devel/setup.bash

echo "=== 1/7 roscore ==="
roscore & sleep 3

echo "=== 2/7 use_sim_time (BEFORE any node) ==="
rosparam set /use_sim_time true

echo "=== 3/7 URDF ==="
xacro --inorder $HOME/catkin_ws_ov/src/open_vins/ov_msckf/gazebo_models/urdf/turtlebot3_waffle_openvins.urdf.xacro > /tmp/robot.urdf 2>/dev/null
python3 -c "
import rospy; rospy.init_node('u',anonymous=True)
rospy.set_param('/robot_description', open('/tmp/robot.urdf').read())"
rosrun robot_state_publisher robot_state_publisher _publish_frequency:=50.0 & sleep 1

echo "=== 4/7 Static TF (timeless, NO global->odom) ==="
python3 "$D/dyn_odom_tf.py" &   # dynamic global->odom from VIO
python3 -c "
import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped
rospy.init_node('static_odom_base')
t = TransformStamped(); t.header.stamp = rospy.Time(0)
t.header.frame_id='odom'; t.child_frame_id='base_footprint'
t.transform.translation.z=-0.078; t.transform.rotation.w=1.0
tf2_ros.StaticTransformBroadcaster().sendTransform(t)
rospy.spin()" &
sleep 1

echo "=== 5/7 pseudo_stereo + OpenVINS ==="
python3 "$D/pseudo_stereo.py" _baseline:=0.08 &
sleep 2
rosrun ov_msckf run_subscribe_msckf _verbosity:=INFO \
  _config_path:=$HOME/catkin_ws_ov/src/open_vins/config/rgbd_stereo/estimator_config.yaml \
  _use_stereo:=true _max_cameras:=2 _publish_global_to_imu_tf:=false __name:=ov_msckf &
sleep 3

echo "=== 6/7 RTAB-Map (launch file for correct odom remap) ==="
rm -f $HOME/.ros/rtabmap.db
roslaunch $HOME/catkin_ws_ov/src/open_vins/ov_msckf/launch/rtabmap_db.launch & sleep 8

echo "=== 7/7 Verify ==="
source /opt/ros/noetic/setup.bash
for n in ov_msckf rtabmap pseudo_stereo dyn_odom_tf; do
  rosnode ping -c 1 /$n 2>/dev/null | head -1
done
echo "use_sim_time=$(rosparam get /use_sim_time)"

echo ""
echo "============================================"
echo " Pipeline ready. Play the bag NOW:"
echo "   rosbag play $BAG --clock \\"
echo "     --topics /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info /imu /clock"
echo ""
echo " After bag finishes:"
echo "   rosrun map_server map_saver map:=/rtabmap/grid_map -f $OUT"
echo "   rtabmap-export --cloud --output ${OUT}_cloud ~/.ros/rtabmap.db"
echo "============================================"
