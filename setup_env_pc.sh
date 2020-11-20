# NOTE: Update the following lines based on your setting
export J7_IP_ADDR=192.168.1.32
export PC_IP_ADDR=192.168.1.20
export CATKIN_WS=$HOME/j7ros_home/catkin_ws

# ROS network setting
export ROS_MASTER_URI=http://$J7_IP_ADDR:11311
export ROS_IP=$PC_IP_ADDR

source /opt/ros/melodic/setup.bash
source $CATKIN_WS/devel/setup.bash
