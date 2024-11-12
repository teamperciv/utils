#!/bin/bash

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
setup_perciv_shell+="export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enp3s0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'"




# Start a new tmux session and created required splits
tmux new-session -d -s $PERCIV_TMUX_NAME
tmux rename-window -t $PERCIV_TMUX_NAME "scripts"

commands=(
    "python3 steering/ros2_controller.py"
    "python3 safety_checking/collision_detector.py"

    # "ros2 run trajectory_prediction trajectory_prediction"
    # "cd overlay_generator/src && python detection_visualizer.py"
    # "cd yolo_detection && python test_pose.py"

    "ros2 launch realsense2_camera rs_multi_camera_launch.py serial_no1:=\'944622073415\' serial_no2:=\'819112073090\' rgb_camera.profile1:=1280,720,30 rgb_camera.profile2:=1280,720,30 depth_module.profile1:=1280,720,30 depth_module.profile2:=1280,720,30 pointcloud.enable1:=false pointcloud.enable2:=false align_depth.enable1:=true align_depth.enable2:=true decimation_filter.enable1:=true decimation_filter.enable2:=true spatial_filter.enable1:=true spatial_filter.enable2:=true temporal_filter.enable1:=true temporal_filter.enable2:=true json_file_path1:=/home/brain/Desktop/MidDensityPreset.json json_file_path2:=/home/brain/Desktop/MidDensityPreset.json enable_infra11:=true enable_infra21:=true enable_infra12:=true enable_infra22:=true camera_name1:=camera1 camera_name2:=camera2"

    # "cd ~/Desktop/FVD_perception_pipeline && python3 rv_bev_stitched_depth.py"
    # For integration with Shahram's UX stuff changed above to this:
    "cd ~/Desktop/FVD_perception_pipeline && python3 rv_bev_stitched_depth_nb.py"
    "cd ~/Desktop/FVD_perception_pipeline && python3 apriltag_node.py"
    "cd ~/Desktop/FVD_perception_pipeline && python3 3d_deproject.py"
    "cd ~/Desktop/Detector_FVD/detection && python3 test_pose.py"

    "cd /home/brain/perciv_ws/src/aruco_pose_estimation/scripts && python3 pose_converter.py"

    "rviz2"
    "ros2 launch car_sim_gazebo world.launch.py"
    # "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True"
    # "ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/perciv_navigation_ws/src/perciv-sim/src/maps/map.yaml use_sim_time:=True"
    "echo $HOME"
    # "ros2 service call /reset_odom std_srvs/srv/Empty '{}'"
)

pane_index=0
for command in "${commands[@]}"; do
    tmux send-keys -t $PERCIV_TMUX_NAME:0.$pane_index "$setup_perciv_shell && $command" C-m
    tmux split-window -v
    tmux select-layout -t $PERCIV_TMUX_NAME tiled
    ((pane_index++))
done

# # Run rviz in another pane since it doesn't need to be actively monitored
# tmux new-window -t $PERCIV_TMUX_NAME -n "RVIZ"
# tmux send-keys -t $PERCIV_TMUX_NAME:1 "$setup_perciv_shell && ros2 run rviz2 rviz2 -d utils/perciv_rviz.rviz" C-m

# bash "$SCRIPT_DIR/start_bev.sh"

# Attach to the tmux session
tmux attach-session -t $PERCIV_TMUX_NAME:0
