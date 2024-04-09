#!/bin/bash

setup_bev_shell="conda activate stitcher && "
setup_bev_shell+="cd ~/Desktop/perciv_svd/SVD_bev_stitched"

allow_ros1="source /opt/ros/noetic/setup.bash"
allow_ros2="conda deactivate && source /opt/ros/foxy/setup.bash"

tmux new-session -d -s PERCIV_BEV_GENERATION

tmux send-keys -t PERCIV_BEV_GENERATION:0.0 "$setup_bev_shell && $allow_ros1 && roscore" C-m
tmux split-window -t PERCIV_BEV_GENERATION:0 -v

wait 2

tmux send-keys -t PERCIV_BEV_GENERATION:0.1 "$setup_bev_shell && $allow_ros1 && streamlit run pub_rs_multi_dep.py" C-m
tmux split-window -t PERCIV_BEV_GENERATION:0 -v

tmux send-keys -t PERCIV_BEV_GENERATION:0.2 "$allow_ros1 && $allow_ros2 && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" C-m

tmux select-layout -t PERCIV_BEV_GENERATION:0 tiled
