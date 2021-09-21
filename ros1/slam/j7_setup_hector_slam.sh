#!/bin/bash
cd $ROS_WS/src/jacinto_ros_perception/ros1/slam
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
cd  hector_slam
git checkout tags/0.5.2 -b j7_hector_slam
git apply $ROS_WS/src/jacinto_ros_perception/ros1/slam/patch/j7_docker_hector_slam.patch
