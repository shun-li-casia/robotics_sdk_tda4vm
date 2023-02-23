#!/bin/bash
# USAGE: mv_log_files.sh <source_string>
#
# Assume run_regression.sh runs in ${HOME}/j7ros_home/ros_ws
# Collect *.md log files under $DST_DIR as follows:
# perf_logs_rosX_<source_string>
# ├── app_estop.md
# ├── app_objdet.md
# ├── app_sde.md
# ├── app_semseg.md
# ├── app_visloc.md
# ├── estop
# │   ├── Log0.md
# │   ├── Log10.md
# │   ├── Log11.md
# │   ├── Log12.md
# │   ├── Log1.md
# │   ├── Log2.md
# │   ├── Log3.md
# │   ├── Log4.md
# │   ├── Log5.md
# │   ├── Log6.md
# │   ├── Log7.md
# │   ├── Log8.md
# │   └── Log9.md
# ├── objdet
# │   ├── Log0.md
# │   ├── Log1.md
# │   ├── Log2.md
# │   └── Log3.md

DST_NAME=perf_logs_ros${ROS_VERSION}
if [[ $# -eq 1 ]]; then
    DST_NAME=${DST_NAME}_$1
fi
DST_DIR=${HOME}/j7ros_home/$DST_NAME

CWD=`pwd`
if [ "$ROS_VERSION" == "1" ]; then
    mv ${HOME}/perf_logs $DST_DIR
    mv ${HOME}/.ros/*.md $DST_DIR
    mv $CWD/*.log $DST_DIR
elif [ "$ROS_VERSION" == "2" ]; then
    mv $CWD/../perf_logs $DST_DIR
    mv $CWD/*.md $DST_DIR
    mv $CWD/*.log $DST_DIR
fi

cd $DST_DIR/..
tar czvf ${DST_NAME}.tar.gz $DST_NAME
cd $CWD