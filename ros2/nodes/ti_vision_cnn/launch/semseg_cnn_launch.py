import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ti_vision_cnn'),
        'config',
        'semseg_params.yaml'
    )

    params = [
        config,
        {
            "input_topic_name":        "camera/image_raw",
            "rectified_image_topic":   "camera/image_rect_nv12",
            "vision_cnn_tensor_topic": "vision_cnn/tensor"
        },
    ]

    node = Node(
        package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    ld.add_action(node)

    return ld
