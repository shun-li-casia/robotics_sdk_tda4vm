import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch for for USB mono camera capture node.
# Requirement: run camera calibration and LDC look-up-table generation tool
# which are located under <mono_cature>/scripts for generating camera_info
# .yaml file and undistortion/rectification look-up table files for
# LDC hardware accelerator

def finslize_node(context, *args, **kwargs):
    model_str   = LaunchConfiguration("model_str").perform(context)
    camera_mode = LaunchConfiguration("camera_mode").perform(context)
    camera_info_yaml  = model_str + "_" + camera_mode + "_camera_info.yaml"

    topic_ns          = LaunchConfiguration("topic_ns").perform(context)
    image_topic       = topic_ns + "/image_raw"
    camera_info_topic = topic_ns + "/camera_info"

    params = [
        {"camera_mode":            LaunchConfiguration("camera_mode")},
        {"frame_rate":             LaunchConfiguration("frame_rate")},
        {"frame_id":               "camera_frame"},
        {"device_name":            LaunchConfiguration("device_name")},
        {"encoding":               LaunchConfiguration("encoding")},
        {"image_topic":            image_topic},
        {"camera_info_topic":      camera_info_topic},
        {"camera_info_yaml":       camera_info_yaml}
    ]

    node = Node(package = "mono_capture",
                executable = "mono_capture",
                name = "mono_capture",
                output = "screen",
                parameters = params)

    return [node]


def generate_launch_description():
    ld = LaunchDescription()

    # model_str: arg that can be set from the command line or a default will be used
    model_str = DeclareLaunchArgument(
        name="model_str",
        default_value=TextSubstitution(text="C920"),
        description='Camera model string'
    )

    # device_name: arg that can be set from the command line or a default will be used
    device_name = DeclareLaunchArgument(
        name="device_name",
        default_value=TextSubstitution(text="/dev/video1"),
        description="""to find your device name, use ls /dev/video*
                       and look for the name begin with video"""
    )

    # camera_mode: arg that can be set from the command line or a default will be used
    camera_mode = DeclareLaunchArgument(
        name="camera_mode",
        default_value=TextSubstitution(text="HD"),
        description="""camera_mode: 'FHD' - 1920x1080, 'HD' - 1280x720, 'VGA' - 640x480"""
    )

    # frame_rate: arg that can be set from the command line or a default will be used
    frame_rate = DeclareLaunchArgument(
        name="frame_rate",
        default_value=TextSubstitution(text="10.0"),
        description='Frame rate'
    )

    # encoding: arg that can be set from the command line or a default will be used
    encoding = DeclareLaunchArgument(
        name="encoding",
        default_value=TextSubstitution(text="yuv422"),
        description='''encoding: "yuv422" (default, output in YUV422:UYVY format), "bgr8"'''
    )

    # topic_ns: arg that can be set from the command line or a default will be used
    topic_ns = DeclareLaunchArgument(
        name="topic_ns",
        default_value=TextSubstitution(text="camera"),
        description='''topic_ns: topic namespace'''
    )

    ld.add_action(model_str)
    ld.add_action(device_name)
    ld.add_action(camera_mode)
    ld.add_action(frame_rate)
    ld.add_action(encoding)
    ld.add_action(topic_ns)
    ld.add_action(OpaqueFunction(function=finslize_node))

    return ld

