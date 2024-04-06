#!/bin/bash

PERCIV_VENV_PATH='/home/shahram95/perciv_venv/bin/activate'
PERCIV_WS_PATH='/home/shahram95/perciv_ws/src'
PERCIV_ROS_PATH='/opt/ros/foxy/setup.bash'
PERCIV_ROS_DOMAIN_ID=100

# Common perciv stuff
make_perciv="conda deactivate && source $PERCIV_VENV_PATH &&source $PERCIV_ROS_PATH && cd $PERCIV_WS_PATH"

echo 'Launching PERCIV demo'

# Source ROS and PERCIV workspace
source $PERCIV_ROS_PATH
source $PERCIV_WS_PATH/install/setup.bash

# source /opt/ros/humble/setup.bash
# source /home/perciv/khush/ros2_ws/install/setup.bash

# Set up environment variables
export ROS_DOMAIN_ID=$PERCIV_ROS_DOMAIN_ID

# Start a new tmux session named 'MySession' detached
tmux new-session -d -s PERCIV_DEMO

# Split the first window vertically thrice
tmux split-window -h

# Split the first pane vertically twice
tmux split-window -v
tmux split-window -v

# Split the second pane horizontally
tmux select-pane -t 0

# Split the second pane vertically twice
tmux split-window -v
tmux split-window -v

# Start reading the joystick
tmux send-keys -t PERCIV_DEMO:0.0 "$make_perciv && python3 steering/ros2_controller.py" C-m

# Start collision detector
tmux send-keys -t PERCIV_DEMO:0.1 "$make_perciv && python3 safety_checking/collision_detector.py" C-m

# Start trajectory predicition node
tmux send-keys -t PERCIV_DEMO:0.2 "$make_perciv && ros2 run trajectory_prediction trajectory_prediction" C-m

# Start rviz
tmux send-keys -t PERCIV_DEMO:0.3 "$make_perciv && ros2 run rviz2 rviz2" C-m

# Start YOLO detections
tmux send-keys -t PERCIV_DEMO:0.4 "$make_perciv && cd overlay_generator/src && python detection_visualizer.py" C-m

# Start overlay generation
tmux send-keys -t PERCIV_DEMO:0.5 "$make_perciv && cd yolo_detection && python test_pose.py" C-m

# Attach to the tmux session
tmux attach -t PERCIV_DEMO
