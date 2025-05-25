#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
cd $(dirname "$0")
colcon build --symlink-install --packages-select goal_remap