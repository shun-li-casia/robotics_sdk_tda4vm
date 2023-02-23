#!/bin/bash
# Usage: ./scripts_internal/yaml_set_field.sh <ros_ver> <param> <value>
# Usage example: ./scripts_internal/yaml_set_field.sh 1 exportPerfStats 1
ROS_VER=$1
FIELD=$2
VALUE=$3

Files_ROS1=(
    ros1/nodes/ti_vision_cnn/config/semseg_params.yaml
    ros1/nodes/ti_vl/config/params.yaml
    ros1/nodes/ti_estop/config/params.yaml
    ros1/nodes/ti_vision_cnn/config/objdet_params_c920.yaml
    ros1/nodes/ti_vision_cnn/config/objdet_params.yaml
    ros1/nodes/ti_vision_cnn/config/semseg_params_c920.yaml
    ros1/nodes/ti_vision_cnn/config/semseg_params.yaml
    ros1/nodes/ti_sde/config/params.yaml
)

Files_ROS2=(
    ros2/nodes/ti_vl/config/params.yaml
    ros2/nodes/ti_estop/config/params.yaml
    ros2/nodes/ti_vision_cnn/config/objdet_params_c920.yaml
    ros2/nodes/ti_vision_cnn/config/objdet_params.yaml
    ros2/nodes/ti_vision_cnn/config/semseg_params_c920.yaml
    ros2/nodes/ti_vision_cnn/config/semseg_params.yaml
    ros2/nodes/ti_sde/config/params.yaml
    ros2/nodes/ti_sde/config/params_pcl.yaml
)

if [ "$ROS_VER" == "1" ]; then
    Files=("${Files_ROS1[@]}")
elif [ "$ROS_VER" == "2" ]; then
    Files=("${Files_ROS2[@]}")
fi

for File in ${Files[@]}; do
    echo "Setting \"${FIELD}: ${VALUE}\" in ${File}"
	sed -i "s/^\(\s*${FIELD}\s*:\s*\).*/\1${VALUE}/" "${File}"
done

if [ "$ROS_VER" == "2" ]; then
    echo "Don't forget to run colcon build to install the updated .yaml files."
done