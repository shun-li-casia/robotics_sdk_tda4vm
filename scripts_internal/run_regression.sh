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

#
# USAGE run_test_with_timeout.sh <timeout> <ROS_VERSION> <RUN_CAMERA_TESTS>
#

# Validate the commandline argument count
if [[ $# -lt 5 ]]
then
    printf "USAGE: $0 <timeout> <LOOP_COUNT> <ROS_VERSION> <RUN_BAG_TESTS> <RUN_CAMERA_TESTS> <FLAG_PERFSTATS> [<ZED_SN>]\n"
    printf "       where ROS_VERSION=[1:ROS1, 2:ROS2]\n"
    printf "             RUN_BAG_TESTS=[0:NO, 1:YES]\n"
    printf "             RUN_CAMERA_TESTS=[0:NO, 1:MONO, 2:STEREO]\n"
    printf "             FLAG_PERFSTATS=[0:PERF_STATS_OFF, 1:PERF_STATS_ON]\n"
    printf "             ZED_SN=SNxxxxxxxx. This is only needed if RUN_CAMERA_TESTS=2\n"
    exit 1
fi

args=($@)
len=${#args[@]}

# First argument is a timeout value
timeout=$1

# Second argument is a loop count
loopCnt=$2

# ROS version
rosVer=$3

# BAG test control
runBagTests=$4

# Camera test control
runCameraTests=$5

# exportPerfStats control
flagPerfStats=$6

# ZED camera serial number
zed_sn=$7

if [ $rosVer != 1 ] && [ $rosVer != 2 ]
then
    echo "Allowed values for ROS_VERSION is [1, 2]"
    exit 2
fi

if [ $runBagTests != 0 ] && [ $runBagTests != 1 ]
then
    echo "Allowed values for RUN_BAG_TESTS is [0, 1]"
    exit 3
fi

if [ $runCameraTests != 0 ] && [ $runCameraTests != 1 ] && [ $runCameraTests != 2 ]
then
    echo "Allowed values for RUN_CAMERA_TESTS is [0, 1, 2]"
    exit 4
fi

if [ $runCameraTests == 2 ] && [ -z $zed_sn ]
then
    echo "Please specify a valid ZED camera serian number in SNxxxxxxxx format."
    exit 5
fi

BOOL_STR=("NO" "YES")
CAM_STR=("NO" "MONO" "STEREO")

BLINK_BEGIN='\033[33m'
BLINK_END='\033[0m'
BRIGHTWHITE='\033[0;37;1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NOCOLOR='\033[0m'

printf "$BLUE"
printf "TEST TIMEOUT VALUE: $timeout \n"
printf "TEST LOOP COUNT   : $loopCnt \n"
printf "ROS_VERSION       : $rosVer \n"
printf "RUN_BAG_TESTS     : ${BOOL_STR[$runBagTests]} \n"
printf "RUN_CAMERA_TESTS  : ${CAM_STR[$runCameraTests]} \n"
printf "$NOCOLOR\n"

ROS1_BAG_TESTS=(
    "roslaunch ti_objdet_range bag_objdet_range.launch"
    "roslaunch ti_vision_cnn bag_semseg_cnn.launch"
    "roslaunch ti_vision_cnn bag_objdet_cnn.launch"
    "roslaunch ti_sde bag_sde.launch"
    "roslaunch ti_sde bag_sde_pcl.launch"
    "roslaunch ti_estop bag_estop.launch"
    "roslaunch ti_vl bag_visloc.launch"
)

ROS1_MONO_CAM_TESTS=(
    "roslaunch ti_vision_cnn gscam_semseg_cnn.launch"
    "roslaunch ti_vision_cnn gscam_objdet_cnn.launch"
    # "roslaunch gscam v4l_mjpg.launch"
    # "roslaunch gscam v4l_yuv.launch"
    # "roslaunch mono_capture mono_capture.launch"
)

ROS1_STEREO_CAM_TESTS=(
    "roslaunch ti_objdet_range zed_objdet_range.launch zed_sn:=${zed_sn}"
    "roslaunch ti_sde zed_sde.launch zed_sn:=${zed_sn}"
    "roslaunch ti_sde zed_sde_pcl.launch zed_sn:=${zed_sn}"
    "roslaunch ti_vision_cnn zed_semseg_cnn.launch zed_sn:=${zed_sn}"
    "roslaunch ti_vision_cnn zed_objdet_cnn.launch zed_sn:=${zed_sn}"
    "roslaunch ti_estop zed_estop.launch zed_sn:=${zed_sn}"
    # "roslaunch zed_capture zed_capture.launch"
)

ROS2_BAG_TESTS=(
    "ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py"
    "ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py"
    "ros2 launch ti_sde bag_sde_launch.py"
    "ros2 launch ti_sde bag_sde_pcl_launch.py"
    "ros2 launch ti_estop bag_estop_launch.py"
)

ROS2_MONO_CAM_TESTS=(
    "ros2 launch ti_vision_cnn gscam_semseg_cnn_launch.py"
    "ros2 launch ti_vision_cnn gscam_objdet_cnn_launch.py"
    # "ros2 launch mono_capture mono_capture_launch.py"
    # "ros2 launch gscam2 v4l_mjpg_launch.py"
    # "ros2 launch gscam2 v4l_yuv_launch.py"
)

ROS2_STEREO_CAM_TESTS=(
    "ros2 launch ti_sde zed_sde_launch.py zed_sn:=${zed_sn}"
    "ros2 launch ti_sde zed_sde_pcl_launch.py zed_sn:=${zed_sn}"
    "ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py zed_sn:=${zed_sn}"
    "ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py zed_sn:=${zed_sn}"
    "ros2 launch ti_estop zed_estop_launch.py zed_sn:=${zed_sn}"
    # "ros2 launch zed_capture zed_capture_launch.py"
)

tests=()

if [ $rosVer == 1 ]; then
    if [ $runBagTests == 1 ]
    then
        tests=("${ROS1_BAG_TESTS[@]}")
    fi

    if [ $runCameraTests == 1 ]
    then
        tests=("${tests[@]}" "${ROS1_MONO_CAM_TESTS[@]}")
    elif [ $runCameraTests == 2 ]
    then
        tests=("${tests[@]}" "${ROS1_STEREO_CAM_TESTS[@]}")
    fi
else
    if [ $runBagTests == 1 ]
    then
        tests=("${ROS2_BAG_TESTS[@]}")
    fi

    if [ $runCameraTests == 1 ]
    then
        tests=("${tests[@]}" "${ROS2_MONO_CAM_TESTS[@]}")
    elif [ $runCameraTests == 2 ]
    then
        tests=("${tests[@]}" "${ROS2_STEREO_CAM_TESTS[@]}")
    fi
fi

testCount=${#tests[@]}

keys=("bag" "mono" "gscam" "zed" "v4l_mjpg" "v4l_yuv" "capture" "semseg" "objdet" "sde" "pcl" "estop" "visloc" "range")

for ((t=0; t<$testCount; t++)); do
    cmd=${tests[t]}
    if [[ "$flagPerfStats" == "1" ]]; then
        cmd+=" exportPerfStats:=1"
    fi

    printf "The command $BLINK_BEGIN[$cmd]$BLINK_END will be run for "
    printf "$BLINK_BEGIN[$timeout seconds]$BLINK_END for "
    printf "a total of $BLINK_BEGIN[$loopCnt]$BLINK_END times.\n"

    name=""
    for ((i=0; i<${#keys[@]}; i++)); do
        case $cmd in
            *"${keys[i]}"*)
              name=$name"_"${keys[i]}
              ;;
        esac
    done

    # Get the current time
    cur_time=$(date "+%Y_%m_%d-%H.%M.%S")

    # Setup log files for stout and stderr for each test run
    stdout="ros${rosVer}${name}-${cur_time}-stdout.log"
    stderr="ros${rosVer}${name}-${cur_time}-stderr.log"

    echo "Directing STDOUT to: $stdout"
    echo "Directing STDERR to: $stderr"

    for ((i=0; i<$loopCnt; i++)); do
        printf "$BLINK_BEGIN[TEST RUN $i] RUNNING $BLINK_END $NOCOLOR\n"
        timeout -s SIGINT -k $(($timeout + 30)) $timeout $cmd >> $stdout 2> $stderr
        test_status=$?

        if [ "$test_status" -ne "0" ] && [ "$test_status" -ne "124" ]
        then
           # Test failed while running
           printf "$BRIGHTWHITE[TEST RUN $i]$RED FAIL $NOCOLOR\n"
        else
           printf "$BRIGHTWHITE[TEST RUN $i]$GREEN PASS $NOCOLOR\n"
        fi

        # Make sure to write the file content to the card
        sync
    done # loop count

    cnt=`grep "APP: Deinit ... Done" $stdout | wc -l`

    if [ $cnt != $loopCnt ]; then
        printf "$BRIGHTWHITE[OVERALL STATUS]$RED EXPECTING $loopCnt SUCCESSFUL RUNS. FOUND ONLY $cnt RUNS. $NOCOLOR\n"
    else
        printf "$BRIGHTWHITE[OVERALL STATUS]$GREEN EXPECTING $loopCnt SUCCESSFUL RUNS. FOUND ALL $cnt RUNS. $NOCOLOR\n"
    fi

    # Trick for "objdet_range": rename output folder/files
    # WARNING: "objdet_range" should come before "objdet" in the test set.
    # Otherwise script below renames more than necessary
    if [[ $cmd == *"objdet_range"* ]]; then
        if [ "$ROS_VERSION" == "1" ]; then
            mv ${HOME}/perf_logs/objdet ${HOME}/perf_logs/objdet_range
            CWD=`pwd`
            cd ${HOME}/.ros
            for file in *_objdet.md; do
                mv "$file" "${file/_objdet/_objdet_range}"
            done
            cd $CWD
        elif [ "$ROS_VERSION" == "2" ]; then
            mv ../perf_logs/objdet ../perf_logs/objdet_range
            for file in *_objdet.md; do
                mv "$file" "${file/_objdet/_objdet_range}"
            done
        fi
    fi

done # test count

