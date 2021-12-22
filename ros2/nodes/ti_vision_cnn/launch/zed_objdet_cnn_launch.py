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

    pkg_dir    = get_package_share_directory('ti_vision_cnn')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include OBJDET launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'objdet_cnn_launch.py')
        )
    )

    # Include ZED launch file
    zed_launch_file = get_launch_file(
        pkg='zed_capture',
        file_name='zed_capture_launch.py'
    )
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={'topic_ns_right': 'camera'}.items()
    )

    ld.add_action(cnn_launch)
    ld.add_action(zed_launch)

    return ld

