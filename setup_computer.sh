#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "$SCRIPT_DIR/env_vars.sh"

if command -v $PERCIV_PYTHON_VERSION &>/dev/null; then
    echo "Creating virtual environment"
else
    echo "Error: $PERCIV_PYTHON_VERSION is not installed."
    exit 1
fi

# Create a virtual environment with the specified Python version
$PERCIV_PYTHON_VERSION -m venv "$PERCIV_VENV_PATH"

# Activate the virtual environment
source $PERCIV_VENV_SOURCE_PATH

# Install packages from requirements.txt
pip install -r "$SCRIPT_DIR/requirements.txt"
pip install catkin_pkg

# Detect ROS version
if [ -d "/opt/ros/foxy" ]; then
    perciv_ros_version="foxy"
elif [ -d "/opt/ros/humble" ]; then
    perciv_ros_version="humble"
else
    echo "Error: Unknown ROS version or ROS not installed"
    exit 1  # Exit with a non-zero status to indicate an error
fi

# Make perciv workspace
mkdir -p "$PERCIV_WS_PATH/src"
cd $PERCIV_WS_PATH
colcon build --symlink-install
source "$PERCIV_WS_PATH/install/setup.bash"

echo "Cloning required repos"
cd "$PERCIV_WS_PATH/src"

git clone git@github.com:teamperciv/yolo_detection.git
git clone git@github.com:teamperciv/overlay_generator.git
git clone git@github.com:teamperciv/steering.git
git clone git@github.com:teamperciv/trajectory_prediction.git
git clone git@github.com:teamperciv/safety_checking.git
git clone git@github.com:teamperciv/utils.git

echo "Building packages"
colcon build --symlink-install
source "$PERCIV_WS_PATH/install/setup.bash"

echo "Computer setup successful!"