import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

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

    # Include rosbag launch file
    # The rosbag launch is located under ti_sde package
    pkg_dir    = get_package_share_directory('ti_sde')
    launch_dir = os.path.join(pkg_dir, 'launch')
    bag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rosbag_remap_launch.py')
        )
    )

    ld.add_action(cnn_launch)
    ld.add_action(bag_launch)

    return ld
