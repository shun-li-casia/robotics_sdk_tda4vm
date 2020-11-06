#!/bin/bash
set -e

# set up ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# set up TI Processor SDK environment
ln -s /host/usr/lib/libtivision_apps.so.7.1.0 /usr/lib/libtivision_apps.so.7.1.0
ln -s /usr/lib/libtivision_apps.so.7.1.0 /usr/lib/libtivision_apps.so
ln -s /host/usr/include/processor_sdk /usr/include/processor_sdk
ln -s /host/usr/lib/libion.so.1.0.0 /usr/lib/libion.so.1.0.0
ln -s /usr/lib/libion.so.1.0.0 /usr/lib/libion.so
ln -s /host/usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so.1.0.0
ln -s /usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so.1
ln -s /usr/lib/libgbm.so.1.0.0 /usr/lib/libgbm.so
ln -s /host/usr/lib/libti_rpmsg_char.so.0.1.0 /usr/lib/libti_rpmsg_char.so.0.1.0
ln -s /usr/lib/libti_rpmsg_char.so.0.1.0 /usr/lib/libti_rpmsg_char.so.0
ln -s /usr/lib/libti_rpmsg_char.so.0.1.0 /usr/lib/libti_rpmsg_char.so

# ROS network settings
export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
export ROS_IP=$J7_IP_ADDR

# source catkin_ws setup.bash if exists
SETUP_FILE=$CATKIN_WS/devel/setup.bash
if [ -f $SETUP_FILE ]; then
    source $SETUP_FILE
fi

exec "$@"
