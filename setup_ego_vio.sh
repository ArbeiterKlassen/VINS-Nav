#!/usr/bin/env bash
# ============================================================================
# setup_ego_vio.sh -- Combined workspace environment for VINS-Nav + EGO-Planner
#
# This script sets up a unified ROS environment that includes BOTH:
#   1. catkin_ws_ov  (OpenVINS, pseudo_stereo, RTAB-Map)
#   2. ego-planner   (EGO-Planner from ZJU-FAST-Lab, pre-built)
#
# Usage:
#   source setup_ego_vio.sh
#   roslaunch ov_msckf nav_ego_vio.launch gazebo:=true
#
# Override the workspace locations with environment variables if your layout
# differs:
#   WS_OV=~/my_ws WS_EGO=~/ego/planner source setup_ego_vio.sh
# ============================================================================

# Base ROS
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/noetic/setup.bash
fi

# Resolve this script's directory so the EGO-Planner workspace is found
# relative to the repo, not by an absolute path.
_SELF="${BASH_SOURCE[0]}"
if [ -n "$_SELF" ]; then
    _REPO_ROOT="$(cd "$(dirname "$_SELF")" && pwd)"
else
    _REPO_ROOT="$(pwd)"
fi

# Workspace 1: OpenVINS + VINS-Nav scripts
WS_OV="${WS_OV:-$HOME/catkin_ws_ov}"
if [ -f "$WS_OV/devel/setup.bash" ]; then
    source "$WS_OV/devel/setup.bash" --extend
else
    echo "[WARN] catkin_ws_ov not found at $WS_OV"
fi

# Workspace 2: EGO-Planner (pre-built, overlay on top)
WS_EGO="${WS_EGO:-$_REPO_ROOT/ego-planner/planner}"
if [ -f "$WS_EGO/devel/setup.bash" ]; then
    CATKIN_SETUP_UTIL_ARGS="--extend" source "$WS_EGO/devel/setup.bash"
else
    echo "[WARN] ego-planner workspace not found at $WS_EGO"
fi

# Fix: strip any newlines from ROS_PACKAGE_PATH (the pre-built ego-planner
# workspace injects them, which makes rospack fail to find every package)
export ROS_PACKAGE_PATH=$(echo "${ROS_PACKAGE_PATH:-}" | tr -d '\n\r')

# Ensure ego-planner message types and libraries are on the path
if [ -d "$WS_EGO/devel/lib" ]; then
    export LD_LIBRARY_PATH="$WS_EGO/devel/lib:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$WS_EGO/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
fi

# Also add ov_msckf scripts directory (for vio_odom_bridge.py etc.)
OV_SCRIPTS="$WS_OV/src/open_vins/ov_msckf/scripts"
if [ -d "$OV_SCRIPTS" ]; then
    export PATH="$OV_SCRIPTS:$PATH"
fi

echo "[setup_ego_vio] Combined workspace ready."
echo "  ov_msckf:       $(rospack find ov_msckf 2>/dev/null || echo 'NOT FOUND')"
echo "  ego_planner:    $(rospack find ego_planner 2>/dev/null || echo 'NOT FOUND')"
echo "  map_tools:      $(rospack find map_tools 2>/dev/null || echo 'NOT FOUND')"
echo "  waypoint_gen:   $(rospack find waypoint_generator 2>/dev/null || echo 'NOT FOUND')"
