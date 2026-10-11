#!/usr/bin/env bash
# ============================================================================
# setup_go1_env.sh -- environment for the Unitree Go1 ROS stack
#
# Sources the Go1 catkin workspace (~/go1_ws) on top of the base ROS install.
# Kept separate from setup_ego_vio.sh (which serves the TurtleBot3 / VINS-Nav
# stack) so the two robot stacks cannot contaminate each other's environments.
#
# The workspace is built selectively -- only the packages needed to talk to the
# robot are compiled:
#
#   unitree_legged_sdk    v3.8.6, the Go1 UDP SDK (a catkin package)
#   unitree_legged_msgs   message definitions (HighState, LowState, ...)
#   unitree_legged_real   the ROS <-> robot bridge
#
# Everything else that was downloaded into ~/go1_ws/src (Gazebo models, camera
# SDK, ROS 2 camera package, ...) is left unbuilt on purpose: they target
# Melodic or Noetic-incompatible stacks and would only break the build.
#
# Usage:
#   source go1/setup_go1_env.sh
#   roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
# ============================================================================

# Base ROS
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/noetic/setup.bash
fi

WS_GO1="${WS_GO1:-$HOME/go1_ws}"

if [ -f "$WS_GO1/devel/setup.bash" ]; then
    # --extend so an already-sourced workspace is not rolled back
    source "$WS_GO1/devel/setup.bash" --extend
else
    echo "[setup_go1_env] WARNING: no workspace at $WS_GO1/devel -- build it first:"
    echo "    cd $WS_GO1 && catkin build unitree_legged_real"
    return 1 2>/dev/null || exit 1
fi

echo "[setup_go1_env] Go1 ROS stack ready."
echo "  unitree_legged_real: $(rospack find unitree_legged_real 2>/dev/null || echo 'NOT FOUND')"
echo "  unitree_legged_msgs: $(rospack find unitree_legged_msgs 2>/dev/null || echo 'NOT FOUND')"
echo "  unitree_legged_sdk : $(rospack find unitree_legged_sdk 2>/dev/null || echo 'NOT FOUND')"
