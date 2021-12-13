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

# Release tag info of the current release
GIT_TAG="v0.5.0"
if [ "$#" -eq 1 ]; then
    GIT_TAG=$1
fi
echo "GIT_TAG = $GIT_TAG"

# Git repository
GIT_REPO="https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git"
BRANCH=master

# Define env variables
ARCH=`arch`
export WORK_DIR=$HOME/j7ros_home
export ROS_WS=$WORK_DIR/ros_ws

# Installation path for Robotics SDK
if [[ "$ARCH" == "aarch64" ]]; then
    export SDK_DIR=/opt/robotics_sdk
elif [[ "$ARCH" == "x86_64" ]]; then
    export SDK_DIR=$ROS_WS/src/robotics_sdk
else
    echo "$ARCH is not supported"
    exit 1
fi

# To support previous versions
if [[ "$GIT_TAG" == "v0.5.0" ]] || [[ "$GIT_TAG" == "v0.4.0" ]]; then
    export SDK_DIR=$ROS_WS/src/jacinto_ros_perception
fi

# "git checkout" only if $GIT_TAG does not match with $RECENT_TAG
function git_checkout_with_tag {
    cd $1
    RECENT_TAG=`git describe --tags --abbrev=0`
    if [[ $RECENT_TAG == ${GIT_TAG}* ]]; then
        echo "Robotics SDK: $GIT_TAG is the most up-to-date version on the remote git repository."
    else
        if [ $(git tag -l "$GIT_TAG") ]; then
            git checkout tags/$GIT_TAG -b $GIT_TAG
        else
            echo "\"$GIT_TAG\" does not exist. \"$RECENT_TAG\" is the latest tag on the git repository."
        fi
    fi
}

# "git pull"" to the current folder
function git_pull_to_current_folder {
    git init
    git remote add origin $GIT_REPO
    git fetch origin
    git checkout -b $BRANCH --track origin/master
    echo "$BRANCH checked out."
}

# Install or update the Robotics SDK
mkdir -p $SDK_DIR
CWD=$(pwd)
if [[ -d "$SDK_DIR/.git" ]]; then
    cd $SDK_DIR
    git pull
else
    if [[ "$CWD" == "$SDK_DIR" ]]; then
        git_pull_to_current_folder
    else
        git clone --branch $BRANCH $GIT_REPO $SDK_DIR
    fi
fi
git_checkout_with_tag $SDK_DIR

# Install gscam/gscam2 or reinstall the packages if already exist
if [[ -f "$SDK_DIR/scripts/install_gscam.sh" ]]; then
    bash $SDK_DIR/scripts/install_gscam.sh
fi

# Setup $WORK_DIR
mkdir -p $ROS_WS/src
cd $WORK_DIR
ln -sf $SDK_DIR/docker/Makefile $WORK_DIR/Makefile
if [[ "$ARCH" == "aarch64" ]]; then
    mkdir -p $WORK_DIR/.ros
fi
if [[ "$ARCH" == "x86_64" ]]; then
    ln -sf $SDK_DIR/setup_env_pc.sh $WORK_DIR/setup_env_pc.sh
fi

# On the TDA target
if [[ "$ARCH" == "aarch64" ]]; then
    # Download and install ROSBAG and other files
    ls $WORK_DIR | grep "data"
    if [ "$?" -ne "0" ]; then
        make data_download
    fi

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

echo "Robotics SDK Setup Done!"