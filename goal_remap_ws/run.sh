#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source the built workspace
source $(dirname "$0")/src/goal_remap/install/setup.bash

# Launch the node
ros2 launch goal_remap goal_remap.launch.py mqtt_broker:=localhost mqtt_port:=1883