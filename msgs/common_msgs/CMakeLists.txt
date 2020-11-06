cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
    FILES
    Detection3D.msg
    Disparity.msg
    DriveSpace.msg
    ObjectPos3D.msg
    ObjectPos.msg
    StereoDetection.msg
    )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
