#!/bin/bash
# Example script for camera calibration
# Please run this script after downloading example calibration images.
#   At Ubuntu PC command line (not in Docker container):
#   ~/j7ros_home$ make calib_download

J7ROS_DIR=${HOME}/j7ros_home
if [ "$ROS_VERSION" == "1" ]; then
    PYTHON=python2
else
    PYTHON=python3
fi
if [[ `arch` == "x86_64" ]]; then
    VIZ_FLAG=True
else
    VIZ_FLAG=False
fi
CAMINFO_FILE="../config/C920_HD_camera_info.yaml"
CAMERA_NAME=C920

# camera calibration
${PYTHON} camera_calibration.py \
    -p ${J7ROS_DIR}/data/calib_images_example \
    -e jpg \
    -o ${CAMINFO_FILE} \
    -r 9 -c 7 -s 25 \
    -v ${VIZ_FLAG} \
    -n ${CAMERA_NAME}

# generate rectification map LUT
${PYTHON} generate_rect_map_mono.py -i ${CAMINFO_FILE} -n ${CAMERA_NAME}
