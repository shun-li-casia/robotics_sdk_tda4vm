#!/bin/bash
set -e

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# ROS network settings
export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
export ROS_IP=$J7_IP_ADDR

# source catkin_ws setup.bash if exists
SETUP_FILE=$CATKIN_WS/devel/setup.bash
if [ -f $SETUP_FILE ]; then
    source $SETUP_FILE
fi

