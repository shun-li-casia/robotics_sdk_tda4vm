import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
             get_package_share_directory('ti_sde'),
             'config',
             'params_pcl.yaml'
             )

    params = [
        {"left_input_topic":  "camera/left/image_raw"},
        {"right_input_topic": "camera/right/image_raw"},
        {"camera_info_topic": "camera/right/camera_info"},
        {"disparity_topic":   "camera/disparity/raw"},
        {"point_cloud_topic": "point_cloud"},
        config
    ]

    node = Node(package = "ti_sde",
                executable = "sde",
                name = "sde",
                output = "screen",
                emulate_tty = True,
                parameters = params)

    ld.add_action(node)

    return ld

