#!/bin/bash
set -e

# setup proxy as required
/root/setup_proxy.sh

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ "$ROS_VERSION" == "1" ]; then
    # ROS network settings
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
    export ROS_IP=$J7_IP_ADDR

    # source catkin_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/devel/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi

elif [ "$ROS_VERSION" == "2" ]; then
    # source catkin_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/install/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi

    # exprimental: https://docs.ros.org/en/foxy/Guides/DDS-tuning.html
    sysctl net.ipv4.ipfrag_high_thresh=134217728

else
    echo "Invalid ROS_VERSION"
fi


exec "$@"
