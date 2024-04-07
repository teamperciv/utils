#!/bin/bash

# Make perciv workspace
mkdir -p ~/perciv_ws/src
cd ~/perciv_ws
# TODO: colcon build
source install/setup.bash
# TODO: git clone all_repos
# TODO: colcon build
source install/setup.bash

# Set up virtual environment
python_version="python3.8"

if command -v $python_version &>/dev/null; then
    echo "Creating virtual environment"
else
    echo "Error: $python_version is not installed."
    exit 1
fi

# Create a virtual environment with the specified Python version
$python_version -m venv ~/perciv_venv

# Activate the virtual environment
source ~/perciv_venv/bin/activate

# Install packages from requirements.txt
pip install -r requirements.txt
