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

    # Include SEMSEG launch file
    cnn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'semseg_cnn_launch.py')
        )
    )

    # Include mono_capture launch file
    mono_launch_file = get_launch_file(
        pkg='mono_capture',
        file_name='mono_capture_launch.py'
    )
    mono_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mono_launch_file)
    )

    ld.add_action(cnn_launch)
    ld.add_action(mono_launch)

    return ld

