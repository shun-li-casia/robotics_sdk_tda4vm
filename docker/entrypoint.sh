#!/bin/bash
set -e

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup TI Processor SDK environment
SRC_PATH=$WORK_DIR/catkin_ws/src/jacinto_ros_perception/docker
FILE="$SRC_PATH/setup_ti_processor_sdk.sh"
if [ -f $FILE ]; then
    source $FILE
else
    echo "$FILE does not exist"
fi

# ROS network settings
export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
export ROS_IP=$J7_IP_ADDR

# source catkin_ws setup.bash if exists
SETUP_FILE=$CATKIN_WS/devel/setup.bash
if [ -f $SETUP_FILE ]; then
    source $SETUP_FILE
fi

exec "$@"
