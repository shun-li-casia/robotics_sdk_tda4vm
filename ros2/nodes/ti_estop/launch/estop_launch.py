import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
             get_package_share_directory('ti_estop'),
             'config',
             'params.yaml'
             )

    params = [
        {"left_input_topic_name":    "camera/left/image_raw"},
        {"right_input_topic_name":   "camera/right/image_raw"},
        {"camera_info_topic":        "camera/right/camera_info"},
        {"rectified_image_topic":    "camera/right/image_rect_mono"},
        {"semseg_cnn_tensor_topic":  "vision_cnn/tensor"},
        {"bounding_box_topic":       "detection3D/BB3D"},
        {"raw_disparity_topic_name": "camera/disparity/raw"},
        {"ogmap_topic_name":         "detection3D/ogmap"},
        {"estop_topic_name":         "detection3D/estop"},
        config
    ]

    node = Node(package = "ti_estop",
                executable = "estop",
                name = "estop",
                output = "screen",
                emulate_tty = True,
                parameters = params
            )

    ld.add_action(node)

    return ld
