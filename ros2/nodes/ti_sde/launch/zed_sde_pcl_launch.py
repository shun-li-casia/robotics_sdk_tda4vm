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

    pkg_dir    = get_package_share_directory('ti_sde')
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Include SDE launch file
    sde_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sde_launch.py')),
        launch_arguments={
            "zed_sn": zed_sn,
            "sde_algo_type": "1",
            "enable_pc": "1",
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

    ld.add_action(sde_launch)
    ld.add_action(zed_launch)

    return ld
