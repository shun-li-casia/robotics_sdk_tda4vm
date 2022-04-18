import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    ld = LaunchDescription()

    # Include ESTOP launch file
    estop_launch_file = get_launch_file(pkg='ti_estop', file_name='estop_launch.py')
    estop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(estop_launch_file)
    )

    # Include rosbag launch file
    # The rosbag launch is located under ti_sde package
    bag_launch_file = get_launch_file(pkg='ti_sde', file_name='rosbag_launch.py')
    bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bag_launch_file)
    )

    ld.add_action(estop_launch)
    ld.add_action(bag_launch)

    return ld
