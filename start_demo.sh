#!/bin/bash

echo 'Launching PERCIV demo'

# Move to the source directory
cd /home/perciv/khush/ros2_ws/src/

# Source ROS
source /opt/ros/humble/setup.bash
source /home/perciv/khush/ros2_ws/install/setup.bash

# Set up environment variables
export ROS_DOMAIN_ID=100

# Start a new tmux session named 'MySession' detached
tmux new-session -d -s PERCIV_DEMO

# Split the first window vertically thrice
tmux split-window -v
tmux split-window -v
tmux split-window -v

# Start reading the joystick
tmux send-keys -t PERCIV_DEMO:0.0 'python3 steering/ros2_controller.py' C-m

# Start collision detector
tmux send-keys -t PERCIV_DEMO:0.1 'python3 safety_checkig/collision_detector.py' C-m

# Start trajectory predicition node
tmux send-keys -t PERCIV_DEMO:0.2 'ros2 run trajectory_prediction trajectory_prediction' C-m

# # Start rviz
# tmux send-keys -t PERCIV_DEMO:0.3 'ros2 run rviz2 rviz2' C-m

# Attach to the tmux session
tmux attach -t PERCIV_DEMO
