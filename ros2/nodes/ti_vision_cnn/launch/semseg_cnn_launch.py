import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def finalize_node(context, *args, **kwargs):
    image_format    = int(LaunchConfiguration("image_format").perform(context))
    enable_ldc_node = int(LaunchConfiguration("enable_ldc_node").perform(context))
    lut_file_path   = LaunchConfiguration("lut_file_path").perform(context)
    dl_model_path   = LaunchConfiguration("dl_model_path").perform(context)
    exportPerfStats = int(LaunchConfiguration("exportPerfStats").perform(context))
    print(f"exportPerfStats = {exportPerfStats}")

    params = [
        os.path.join(get_package_share_directory('ti_vision_cnn'),'config','params.yaml'),
        {
            "image_format":             image_format,
            "lut_file_path":            lut_file_path,
            "enable_ldc_node":          enable_ldc_node,
            "dl_model_path":            dl_model_path,
            "input_topic_name":         "camera/image_raw",
            "rectified_image_topic":    "camera/image_rect_nv12",
            "rectified_image_frame_id": "right_frame",
            "vision_cnn_tensor_topic":  "vision_cnn/tensor",
            "exportPerfStats":           exportPerfStats,
        },
    ]

    node = Node(
        package = "ti_vision_cnn",
        executable = "vision_cnn",
        name = "vision_cnn",
        output = "screen",
        emulate_tty = True,
        parameters = params
    )

    return [node]


def generate_launch_description():
    ld = LaunchDescription()

    # Input image format: 0 - VX_DF_IMAGE_U8, 1 - VX_DF_IMAGE_NV12, 2 - VX_DF_IMAGE_UYVY
    image_format = DeclareLaunchArgument(
        name="image_format",
        default_value=TextSubstitution(text='2'),
        description='input image format'
    )

    # enable ldc node flag
    enable_ldc_node = DeclareLaunchArgument(
        name="enable_ldc_node",
        default_value=TextSubstitution(text='1'),
        description='enable ldc node flag'
    )

    # LDC LUT definition file path
    lut_file_path = DeclareLaunchArgument(
        name="lut_file_path",
        default_value=TextSubstitution(text="/opt/robotics_sdk/ros1/drivers/zed_capture/config/SN5867575_HD_LUT_right.bin"),
        description='LDC LUT definition file path'
    )

    # DL model path
    dl_model_path = DeclareLaunchArgument(
        name="dl_model_path",
        default_value=TextSubstitution(text="/opt/model_zoo/TVM-SS-5818-deeplabv3lite-mobv2-qat-robokit-768x432"),
        # default_value=TextSubstitution(text="/opt/model_zoo/ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432"),
        description='DL model path'
    )

    # Flag for exporting the performance data to a file: 0 - disable, 1 - enable
    exportPerfStats = DeclareLaunchArgument(
        name="exportPerfStats",
        default_value=TextSubstitution(text="0"),
        description='flag for exporting the performance data'
    )

    ld.add_action(image_format)
    ld.add_action(enable_ldc_node)
    ld.add_action(lut_file_path)
    ld.add_action(dl_model_path)
    ld.add_action(exportPerfStats)
    ld.add_action(OpaqueFunction(function=finalize_node))

    return ld
