import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
             get_package_share_directory('ti_vl'),
             'config',
             'params.yaml'
             )

    params = [
        {"input_topic":     "camera/image_raw"},
        {"out_image_topic": "vis_localize/out_image"},
        {"out_pose_topic":  "vis_localize/pose"},
        {"map_topic":       "vis_localize/map"},
        config
    ]

    node = Node(package = "ti_vl",
                executable = "vl",
                name = "vl",
                output = "screen",
                parameters = params
            )

    ld.add_action(node)

    return ld
