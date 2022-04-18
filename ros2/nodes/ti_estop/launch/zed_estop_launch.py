import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

# ZED camera serial number
zed_sn = "SN18059"

def get_launch_file(pkg, file_name):
    pkg_dir = get_package_share_directory(pkg)
    return os.path.join(pkg_dir, 'launch', file_name)

def generate_launch_description():
    ld = LaunchDescription()

    # Include SDE launch file
    estop_launch_file = get_launch_file(pkg='ti_estop', file_name='estop_launch.py')
    estop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(estop_launch_file),
        launch_arguments={
            "zed_sn": zed_sn,
        }.items()
    )

    # Include ZED launch file
    zed_launch_file = get_launch_file('zed_capture', 'zed_capture_launch.py')
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            "zed_sn_str": zed_sn,
        }.items()
    )

    ld.add_action(estop_launch)
    ld.add_action(zed_launch)

    return ld
