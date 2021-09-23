# NOTE: Update the following lines based on your setting
export J7_IP_ADDR=192.168.1.32
export PC_IP_ADDR=192.168.1.20
export ROS_WS=${HOME}/j7ros_home/ros_ws

# set up ROS environment
if [ "$ROS_VERSION" == "1" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    # ROS 1 network setting
    export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
    export ROS_IP=$PC_IP_ADDR
    # source ros_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/devel/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi
elif [ "$ROS_VERSION" == "2" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    # source ros_ws setup.bash if exists
    SETUP_FILE=$ROS_WS/install/setup.bash
    if [ -f $SETUP_FILE ]; then
        source $SETUP_FILE
    fi
else
    echo "ROS is not set up or invalid ROS_VERSION"
fi
