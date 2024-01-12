"""
Launch gscam2 node with parameters and remappings.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def finalize_node(context, *args, **kwargs):

    # Device
    device = "/dev/video" + LaunchConfiguration('video_id').perform(context)

    # Framerate
    framerate = LaunchConfiguration('framerate')

    # Image encoding: "yuv420" - publishes in "NV12" (default), "rgb8"
    image_encoding = LaunchConfiguration('image_encoding')

    # Name of the gscam2 node
    node_name = LaunchConfiguration('node_name')

    # GStreamer pipeline specified in gscam_config was tested with OV5640 CSI camera in YUYV mode.
    # Assumes 'edgeai-tiovx-modules' and 'edgeai-gst-plugins' are already installed in the ROS container
    gscam_config = 'v4l2src device=' + device + ' io-mode=0 do-timestamp=true ! tiovxdlcolorconvert'

    # Camera namespace
    camera_name = LaunchConfiguration('camera_name').perform(context)

    # Camera calibration file
    config_dir = os.path.join(get_package_share_directory('gscam2'), 'config')
    camera_config = 'file://' + os.path.join(config_dir, 'C920_HD_camera_info.yaml')
    print(camera_config)

    node = Node(
        package    = 'gscam2',
        executable = 'gscam_main',
        output     = 'screen',
        name       = node_name,
        namespace  = camera_name,
        parameters = [
            {
                'gscam_config':       gscam_config,
                'camera_name':        camera_name,
                'camera_info_url':    camera_config,
                'image_encoding':     image_encoding,
                'appsink_width':      1280,
                'appsink_height':     720,
                'appsink_framerate':  framerate,
                'sync_sink':          False,
                'preroll':            False,
                'use_gst_timestamps': False,
                'frame_id':           'camera_frame',
            },
        ],
        # Remap outputs to the correct namespace
        remappings=[
            ('/image_raw', '/' + camera_name + '/image_raw'),
            ('/camera_info', '/' + camera_name + '/camera_info'),
        ],
    )

    return [node]


def generate_launch_description():

    video_id_arg = DeclareLaunchArgument(
        'video_id',
        default_value='2',
        description='ID of the video device to use.'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Frame rate of the camera'
    )

    image_encoding_arg = DeclareLaunchArgument(
        'image_encoding',
        default_value='yuv420',
        description='Encoding of the camera image'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='gscam_publisher',
        description='Name of the gscam2 node'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera namespace'
    )

    # Create the launch description and add the arguments
    ld  = LaunchDescription()
    ld.add_action(video_id_arg)
    ld.add_action(framerate_arg)
    ld.add_action(image_encoding_arg)
    ld.add_action(node_name_arg)
    ld.add_action(camera_name_arg)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld