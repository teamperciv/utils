#!/bin/bash

# set -e

echo 'Launching PERCIV demo'


PERCIV_TMUX_NAME="PERCIV_AUTONOMY_DEMO"
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

# Set up environment variables
source "$SCRIPT_DIR/env_vars.sh"

# Detect ROS version
if [ -d "/opt/ros/foxy" ]; then
    perciv_ros_version="foxy"
elif [ -d "/opt/ros/humble" ]; then
    perciv_ros_version="humble"
else
    echo "Error: Unknown ROS version or ROS not installed"
    exit 1  # Exit with a non-zero status to indicate an error
fi

PERCIV_ROS_SOURCE_PATH="/opt/ros/$perciv_ros_version/setup.bash"

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
# setup_perciv_shell+="source $PERCIV_VENV_SOURCE_PATH && "
setup_perciv_shell+="source $PERCIV_ROS_SOURCE_PATH && "
setup_perciv_shell+="source $PERCIV_WS_PATH/install/local_setup.bash && "
setup_perciv_shell+="cd $PERCIV_WS_PATH/src && "
setup_perciv_shell+="source $PERCIV_NAV_WS_PATH/install/local_setup.bash && "
setup_perciv_shell+="export ROS_DOMAIN_ID=$PERCIV_ROS_DOMAIN_ID && "
setup_perciv_shell+="export TURTLEBOT3_MODEL=waffle && "
setup_perciv_shell+="export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>en01</NetworkInterfaceAddress></General></Domain></CycloneDDS>'"

# Start a new tmux session and created required splits
tmux new-session -d -s $PERCIV_TMUX_NAME

# commands that don't require monitoring
bg_commands=(
    "ros2 launch realsense2_camera rs_multi_camera_launch.py serial_no1:=\'944622073415\' serial_no2:=\'819112073090\' rgb_camera.profile1:=1280,720,30 rgb_camera.profile2:=1280,720,30 depth_module.profile1:=1280,720,30 depth_module.profile2:=1280,720,30 pointcloud.enable1:=false pointcloud.enable2:=false align_depth.enable1:=true align_depth.enable2:=true decimation_filter.enable1:=true decimation_filter.enable2:=true spatial_filter.enable1:=true spatial_filter.enable2:=true temporal_filter.enable1:=true temporal_filter.enable2:=true json_file_path1:=/home/brain/Desktop/MidDensityPreset.json json_file_path2:=/home/brain/Desktop/MidDensityPreset.json enable_infra11:=true enable_infra21:=true enable_infra12:=true enable_infra22:=true camera_name1:=camera1 camera_name2:=camera2"
    "cd /home/brain/perciv_ws/src/aruco_pose_estimation/scripts && python3 pose_converter.py"

    "cd ~/Desktop/FVD_perception_pipeline && streamlit run interface_v5.py"

    "rviz2"
    "ros2 launch car_sim_gazebo world.launch.py"

    "echo testing terminal"
)

pnc_commands=(
    "python3 steering/ros2_controller.py"
    "python3 safety_checking/collision_detector.py"
    "cd /home/brain/perciv_ws/src/aruco_pose_estimation/scripts && python3 pose_converter.py"

    "ros2 run trajectory_prediction trajectory_prediction"
    "echo testing terminal"
)

perception_commands=(
    # "cd ~/Desktop/FVD_perception_pipeline && python3 rv_bev_stitched_depth.py"
    # For integration with Shahram's UX stuff changed above to this:
    "cd ~/Desktop/FVD_perception_pipeline && python3 rv_bev_stitched_depth_nb.py"
    "cd ~/Desktop/FVD_perception_pipeline && python3 apriltag_node.py"
    "cd ~/Desktop/FVD_perception_pipeline && python3 3d_deproject.py"
    "cd ~/Desktop/Detector_FVD/detection && python3 test_pose.py"

    "echo testing terminal"
)

# "ros2 service call /reset_odom std_srvs/srv/Empty '{}'"

# Function to create and populate tmux window
create_tmux_window() {
    local session=$1
    local window_name=$2
    shift 2
    local commands=("$@")

    tmux new-window -t "$session" -n "$window_name"
    local pane_index=0

    for command in "${commands[@]}"; do
        (( pane_index > 0 )) && tmux split-window -t "$session:$window_name" -v
        tmux send-keys -t "$session:$window_name.$pane_index" "$setup_perciv_shell" C-m
        tmux send-keys -t "$session:$window_name.$pane_index" "$command" C-m
        tmux select-layout -t "$session:$window_name" tiled
        ((pane_index++))
    done
}

# Initialize tmux session and create windows
tmux new-session -d -s "$PERCIV_TMUX_NAME" -n "bg_commands"
create_tmux_window "$PERCIV_TMUX_NAME" "bg_commands" "${bg_commands[@]}"
create_tmux_window "$PERCIV_TMUX_NAME" "pnc_commands" "${pnc_commands[@]}"
create_tmux_window "$PERCIV_TMUX_NAME" "perception_commands" "${perception_commands[@]}"

# Attach to the tmux session
tmux attach-session -t "$PERCIV_TMUX_NAME:pnc_commands"
