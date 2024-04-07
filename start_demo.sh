#!/bin/bash

echo 'Launching PERCIV demo'

USER_HOME=$(eval echo "~$USER")

# Detect ROS version
if [ -d "/opt/ros/foxy" ]; then
    perciv_ros_version="foxy"
elif [ -d "/opt/ros/humble" ]; then
    perciv_ros_version="humble"
else
    echo "Error: Unknown ROS version or ROS not installed"
    exit 1  # Exit with a non-zero status to indicate an error
fi

PERCIV_VENV_SOURCE_PATH="$USER_HOME/perciv_venv/bin/activate"
PERCIV_WS_PATH="$USER_HOME/perciv_ws"
PERCIV_ROS_SOURCE_PATH="/opt/ros/$perciv_ros_version/setup.bash"
PERCIV_ROS_DOMAIN_ID=100

# Sanity checks
if [ ! -f $PERCIV_VENV_SOURCE_PATH ]; then
    echo "Error: PERCIV virtual env not found"
    exit 1
fi
if [ ! -d $PERCIV_WS_PATH ]; then
    echo "Error: PERCIV workspace directory not found"
    exit 1
fi

# Common perciv stuff
setup_perciv_shell="conda deactivate && "
setup_perciv_shell+="source $PERCIV_VENV_SOURCE_PATH && "
setup_perciv_shell+="source $PERCIV_ROS_SOURCE_PATH && "
setup_perciv_shell+="source $PERCIV_WS_PATH/install/setup.bash && "
setup_perciv_shell+="cd $PERCIV_WS_PATH/src"

# Set up environment variables
export ROS_DOMAIN_ID=$PERCIV_ROS_DOMAIN_ID

# Start a new tmux session and created required splits
tmux new-session -d -s PERCIV_DEMO
tmux rename-window -t PERCIV_DEMO "scripts"

commands=(
    "python3 steering/ros2_controller.py"
    "python3 safety_checking/collision_detector.py"
    "ros2 run trajectory_prediction trajectory_prediction"
    "cd overlay_generator/src && python detection_visualizer.py"
    "cd yolo_detection && python test_pose.py"
)

pane_index=0
for command in "${commands[@]}"; do
    tmux send-keys -t PERCIV_DEMO:0.$pane_index "$setup_perciv_shell && $command" C-m
    tmux split-window -v
    tmux select-layout -t PERCIV_DEMO tiled
    ((pane_index++))
done

# Run rviz in another pane since it doesn't need to be actively monitored
tmux new-window -t PERCIV_DEMO -n "RVIZ"
tmux send-keys -t PERCIV_DEMO:1 "$setup_perciv_shell && ros2 run rviz2 rviz2" C-m

# Attach to the tmux session
tmux attach-session -t PERCIV_DEMO:0
