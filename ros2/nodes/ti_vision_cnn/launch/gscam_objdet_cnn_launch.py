import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    ld = LaunchDescription()

    # ti_vision_cnn node
    config = os.path.join(
        get_package_share_directory('ti_vision_cnn'),
        'config',
        'objdet_params_c920.yaml'
    )
    params = [
        config,
        {
            "input_topic_name":        "camera/image_raw",
            "rectified_image_topic":   "camera/image_rect_nv12",
            "vision_cnn_tensor_topic": "vision_cnn/tensor",
        },
    ]
    cnn_node = Node(package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    # Include gscam2 launch file
    cam_launch_file = get_launch_file(
        pkg='gscam2',
        file_name='v4l_mjpg_launch.py'
    )
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_launch_file)
    )

    ld.add_action(cnn_node)
    ld.add_action(cam_launch)

    return ld

