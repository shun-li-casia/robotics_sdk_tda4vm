#!/bin/bash
ROS_WS=$HOME/j7ros_home/ros_ws
ARCH=`arch`
if [[ "$ARCH" == "aarch64" ]]; then
    SDK_DIR=/opt/robotics_sdk
elif [[ "$ARCH" == "x86_64" ]]; then
    SDK_DIR=$ROS_WS/src/robotics_sdk
else
    echo "$ARCH is not supported"
    exit 1
fi

# install mmwave_rospkg
CURRENT_DIR=$(pwd)
WORK_PATH=$SDK_DIR/ros1/drivers
cd $WORK_PATH
if [[ ! -d "ti_mmwave_rospkg" ]]; then
    git clone --single-branch --branch master https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git
    cd mmwave_ti_ros
    git checkout 4d3efaaa476259a0cec09db660a21d48dd9860ca
    cd ..
    cp -r mmwave_ti_ros/ros_driver/src/* .
    rm -rf mmwave_ti_ros
fi
cd $CURRENT_DIR
