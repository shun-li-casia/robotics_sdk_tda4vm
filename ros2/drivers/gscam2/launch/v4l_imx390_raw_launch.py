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

    """
    You can check the device_id and subdev_id for the IMX390 camera
    attached by running /opt/edgeai-gst-apps/scripts/setup_cameras.sh on the target
    host Linux. Accordingly please update the parameters or pass as launch arguments.
    """
    # Device
    device = "/dev/video" + LaunchConfiguration('video_id').perform(context)

    # Subdev
    subdev = "/dev/v4l-subdev" + LaunchConfiguration('subdev_id').perform(context)

    # width and height of the image
    width  = 1936
    height = 1096

    # Framerate
    framerate = LaunchConfiguration('framerate')

    # Image encoding: "yuv420" - publishes in "NV12" (default), "rgb8"
    image_encoding = LaunchConfiguration('image_encoding')

    # Name of the gscam2 node
    node_name = LaunchConfiguration('node_name')

    # sensor_name
    sensor_name = 'SENSOR_SONY_IMX390_UB953_D3'

    # DCC VISS binary file
    dcc_isp_file = '/opt/imaging/imx390/dcc_viss.bin'

    # DCC 2A binary file
    dcc_2a_file = '/opt/imaging/imx390/dcc_2a.bin'

    # camera_info URL: replace with camera_info from camera calibration
    camera_info_url = 'file:///opt/robotics_sdk/tools/mono_camera/imx390_35244_equidistant_camera_info.yaml'

    # GStreamer pipeline specified in gscam_config was tested with IMX390,
    # 'edgeai-tiovx-modules' and 'edgeai-gst-plugins' are already installed in the ROS container
    gscam_config = 'v4l2src device={} do-timestamp=true ! '.format(device) \
        + 'video/x-bayer, width={}, height={}, format=rggb12 ! '.format(width, height) \
        + 'tiovxisp sink_0::device={} dcc-isp-file={} '.format(subdev, dcc_isp_file) \
        + 'sink_0::dcc-2a-file={} format-msb=11 sensor-name={} ! '.format(dcc_2a_file, sensor_name) \
        + 'video/x-raw, format=NV12 ! ' \
        + 'tiovxdlcolorconvert target=1 out-pool-size=4'
    print(gscam_config)

    # Camera namespace
    camera_name = LaunchConfiguration('camera_name').perform(context)

    # camera_info URL: replace with camera_info from camera calibration
    camera_info_url = '/opt/robotics_sdk/tools/mono_camera/imx390_35244_equidistant_camera_info.yaml'
    print(camera_info_url)

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
                'camera_info_url':    camera_info_url,
                'image_encoding':     image_encoding,
                'appsink_width':      width,
                'appsink_height':     height,
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

    subdev_id_arg = DeclareLaunchArgument(
        'subdev_id',
        default_value='2',
        description='ID of subdev to use.'
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
    ld = LaunchDescription()
    ld.add_action(video_id_arg)
    ld.add_action(subdev_id_arg)
    ld.add_action(framerate_arg)
    ld.add_action(image_encoding_arg)
    ld.add_action(node_name_arg)
    ld.add_action(camera_name_arg)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld