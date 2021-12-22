#!/bin/bash
cd /opt/robotics_sdk/ros1/slam
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
cd  hector_slam
git checkout tags/0.5.2 -b j7_hector_slam
git apply /opt/robotics_sdk/ros1/slam/patch/j7_docker_hector_slam.patch
