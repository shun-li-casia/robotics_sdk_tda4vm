import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # color conversion for input image_raw for visualization
    params = [
        {"width":            1280},
        {"height":           720},
        {"input_yuv_topic":  "camera/image_raw"},
        {"output_rgb_topic": "camera/image_raw_rgb"},
        {"yuv_format":       "YUV422"}
    ]
    yuv2rbg_node1 = Node(package = "ti_viz_nodes",
                         executable = "viz_color_conv_yuv2rgb",
                         name = "viz_color_conv_yuv2rgb_input",
                         output = "screen",
                         parameters = params)

    # color conversion for image_rect_nv12 for visualization
    params = [
        {"width":            1280},
        {"height":           720},
        {"input_yuv_topic":  "camera/image_rect_nv12"},
        {"output_rgb_topic": "camera/image_rect_rgb"},
        {"yuv_format":       "YUV420"},
        {"yuv420_luma_only": False}
    ]
    yuv2rbg_node2 = Node(package = "ti_viz_nodes",
                         executable = "viz_color_conv_yuv2rgb",
                         name = "viz_color_conv_yuv2rgb_node_rect",
                         output = "screen",
                         parameters = params)

    # color conversion for image_rect_nv12 for visualization
    params = [
        {"rectified_image_topic":   "camera/image_rect_rgb"},
        {"vision_cnn_tensor_topic": "vision_cnn/tensor"},
        {"vision_cnn_image_topic":  "vision_cnn/out_image"}
    ]
    viz_objdet_node = Node(package = "ti_viz_nodes",
                executable = "viz_objdet",
                name = "viz_objdet",
                output = "screen",
                parameters = params)

    # image_view node
    image_view_node = Node(package = "image_view",
                      executable = "image_view",
                      name = "image_view",
                      output = "screen",
                      parameters = [{"autosize": True}],
                      remappings=[("image", "vision_cnn/out_image")]
                )

    # Create the launch description with launch and node information
    ld.add_action(yuv2rbg_node1)
    ld.add_action(yuv2rbg_node2)
    ld.add_action(viz_objdet_node)
    ld.add_action(image_view_node)

    return ld
