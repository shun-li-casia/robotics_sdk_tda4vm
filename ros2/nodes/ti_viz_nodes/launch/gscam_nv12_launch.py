import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # color conversion for input image_raw for visualization
    params = [
        {
            "width":            1280,
            "height":           720,
            "input_yuv_topic":  "camera/image_raw",
            "output_rgb_topic": "camera/image_raw_rgb",
            "yuv_format":       "YUV420",
            "yuv420_luma_only": False,
        }
    ]
    yuv2rbg_node = Node(
        package = "ti_viz_nodes",
        executable = "viz_color_conv_yuv2rgb",
        name = "viz_color_conv_yuv2rgb_node",
        output = "screen",
        parameters = params
    )

    # image_view node
    image_view_node = Node(
        package = "image_view",
        executable = "image_view",
        name = "image_view",
        output = "screen",
        parameters = [
            {"autosize": True},
        ],
        remappings=[
            ("image", "/camera/image_raw_rgb"),
        ]
    )

    # Create the launch description with launch and node information
    ld.add_action(yuv2rbg_node)
    ld.add_action(image_view_node)

    return ld
