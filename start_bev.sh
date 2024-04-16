#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "$SCRIPT_DIR/env_vars.sh"

setup_bev_shell="conda activate stitcher && "
setup_bev_shell+="cd ~/Desktop/perciv_svd/SVD_bev_stitched"

# Detect ROS version
if [ -d "/opt/ros/foxy" ]; then
    perciv_ros_version="foxy"
elif [ -d "/opt/ros/humble" ]; then
    perciv_ros_version="humble"
else
    echo "Error: Unknown ROS version or ROS not installed"
    exit 1  # Exit with a non-zero status to indicate an error
fi

allow_ros1="source /home/teamc24/ros_noetic_ws/install_isolated/setup.bash"
allow_ros2="conda deactivate && source /opt/ros/$perciv_ros_version/setup.bash"
source_perciv_ws="source $PERCIV_WS_PATH/install/local_setup.bash"

tmux new-session -d -s PERCIV_BEV_GENERATION

tmux send-keys -t PERCIV_BEV_GENERATION:0.0 "$setup_bev_shell && $allow_ros1 && roscore" C-m
tmux split-window -t PERCIV_BEV_GENERATION:0 -v

wait 2

tmux send-keys -t PERCIV_BEV_GENERATION:0.1 "$setup_bev_shell && $allow_ros1 && streamlit run pub_rs_multi_dep.py" C-m
tmux split-window -t PERCIV_BEV_GENERATION:0 -v

tmux send-keys -t PERCIV_BEV_GENERATION:0.2 "$allow_ros1 && $allow_ros2 && $source_perciv_ws && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

tmux select-layout -t PERCIV_BEV_GENERATION:0 tiled
