#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ARCH=`arch`
export WORK_DIR=$HOME/j7ros_home
export ROS_WS=$WORK_DIR/ros_ws

# Install Robotics SDK repository
if [[ ! -d "$ROS_WS/src/jacinto_ros_perception" ]]; then
    mkdir -p $ROS_WS/src
    cd $ROS_WS/src
    git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git
    cd $WORK_DIR
    ln -sf $ROS_WS/src/jacinto_ros_perception/docker/Makefile

    if [[ "$ARCH" == "aarch64" ]]; then
        mkdir -p $WORK_DIR/.ros
    fi
    if [[ "$ARCH" == "x86_64" ]]; then
        ln -sf $ROS_WS/src/jacinto_ros_perception/setup_env_pc.sh $WORK_DIR/setup_env_pc.sh
    fi
fi

if [[ "$ARCH" == "aarch64" ]]; then
    # Download and install ROSBAG and other files
    make data_download

    # Install Tensorflow for CPP apps build
    ls /opt | grep "tensorflow"
    if [ "$?" -ne "0" ]; then
        bash /opt/edge_ai_apps/scripts/install_tensorflow.sh
    fi

    # Install dlpack for CPP apps build
    ls /opt | grep "dlpack"
    if [ "$?" -ne "0" ]; then
        bash /opt/edge_ai_apps/scripts/install_dlpack.sh 0.5
    fi

    # Install ONNX RT for CPP apps build
    ls /opt | grep "onnxruntime"
    if [ "$?" -ne "0" ]; then
        bash /opt/edge_ai_apps/scripts/install_onnx_rt.sh
    fi

    # Download models in the model zoo
    Models=(
        ONR-OD-8080-yolov3-lite-regNetX-1.6gf-bgr-coco-512x512
        TFL-OD-2020-ssdLite-mobDet-DSP-coco-320x320
    )
    for Model in ${Models[@]}; do
	    bash /opt/edge_ai_apps/download_models.sh -d $Model
    done
fi

sync

echo "Setup Done!"