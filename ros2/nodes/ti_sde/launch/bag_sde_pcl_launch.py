import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir    = get_package_share_directory('ti_sde')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include SDE launch file
    sde_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'sde_pcl_launch.py')
                )
            )

    # Include rosbag launch file
    bag_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rosbag_launch.py')
                )
            )

    ld.add_action(sde_launch)
    ld.add_action(bag_launch)

    return ld
