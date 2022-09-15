Vision CNN: Semantic Segmentation
=================================

![](docs/semseg_rviz.png)
<br />

This `ti_vision_cnn` node is versatile deep-learning (DL) inference ROS node that is optimized on DL cores and hardware accelerator of TDA4. The `ti_vision_cnn` node supports compute-intensive DL inference operations including 2D object detection and semantic segmentation. Figure 1 shows the high-level block diagram of the applications around the `ti_vision_cnn` node, which consists of multiple processing blocks that are deployed on hardware accelerators and DSP processors for pre-processing and post-processing in an optimized manner.

![](docs/semseg_demo_block_diagram.svg)
<figcaption>Figure 1. Semantic segmentation demo: block diagram</figcaption>
<br />

## Semantic Segmentation Demo

### Semantic Segmentation Model
A CNN model for semantic segmentation has been developed using [Jacinto AI DevKit (PyTorch)](https://git.ti.com/cgit/jacinto-ai/pytorch-jacinto-ai-devkit/about/).

* Model: deeplabv3lite_mobilenetv2_tv (for details, see [LINK](https://git.ti.com/cgit/jacinto-ai/pytorch-jacinto-ai-devkit/about/docs/Semantic_Segmentation.md))
* Input image: 768 x 432 pixels in RGB
* Training data: [Cityscapes Dataset](https://www.cityscapes-dataset.com), and a small dataset collected from ZED camera
* Model compilation: TVM compilation for generating DLR model artifacts. For more details on open-source deep-learning runtime on TDA4, please check [TI Edge AI Cloud](https://dev.ti.com/edgeai/).

### How to Run the Application in ROS 1

**[TDA4]** To launch semantic segmentation demo with playing back a ROSBAG file, run the following inside the Docker container on TDA4 target:
```
roslaunch ti_vision_cnn bag_semseg_cnn.launch
```
To process the image stream from a ZED stereo camera:
```
roslaunch ti_vision_cnn zed_semseg_cnn.launch zed_sn:=SNxxxxx
```
To process the image stream from a USB mono camera:
```
roslaunch ti_vision_cnn mono_semseg_cnn.launch
# Alternatively
roslaunch ti_vision_cnn gscam_semseg_cnn.launch
```

For IMX390 camera as input, depending on choice of resolution, run one from the following:
```
roslaunch ti_vision_cnn gscam_semseg_cnn_imx390.launch width:=1920 height:=1080
roslaunch ti_vision_cnn gscam_semseg_cnn_imx390.launch width:=1280 height:=720
roslaunch ti_vision_cnn gscam_semseg_cnn_imx390.launch width:=960 height:=540
```

**[Visualization on Ubuntu PC]** For setting up environment of the remote PC, please follow [Docker Setup for ROS 1](../../../docker/setting_docker_ros1.md)

The semantic segmentation output tensor is published by the application. For visualization, a color-coded semantic segmentation image is generated. Following command launches the resulting color-mapped semantic segmentation image along with the raw image on RViz:
```
roslaunch ti_viz_nodes rviz_semseg_cnn.launch
```

For IMX390 camera, depending on the resolution from the TDA4, run one from the following:
```
roslaunch ti_viz_nodes rviz_semseg_cnn.launch width:=1920 height:=1080
roslaunch ti_viz_nodes rviz_semseg_cnn.launch width:=1280 height:=720
roslaunch ti_viz_nodes rviz_semseg_cnn.launch width:=960 height:=540
```

#### Testing Higher Input Frame Rate with ROSBAG
We can publish the images at higher frame rate with `rosbag play --rate` option. Below is an example to publish the raw images at 30 fps (double the native frame rate of the ROSBAG file).
```
roslaunch ti_vision_cnn bag_semseg_cnn.launch ratefactor:=2.0
```

### How to Run the Application in ROS 2

**[TDA4]** To launch semantic segmentation demo with a ZED stereo camera, run the following inside the Docker container on TDA4 target:
```
ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py zed_sn:=SNxxxxx
```
To process the image stream from a USB mono camera:
```
roslaunch ti_vision_cnn mono_semseg_cnn.launch
# Alternatively
roslaunch ti_vision_cnn gscam_semseg_cnn.launch
```
<!-- To launch semantic segmentation demo with playing back a ROSBAG file, run the following inside the Docker container on TDA4 target:
```
ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py
``` -->

Launch example for IMX390 camera is to be added later for ROS 2.

**[Visualization on Ubuntu PC]** For setting up environment of the remote PC, please follow [Docker Setup for ROS 2](../../../docker/setting_docker_ros2.md)

```
ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
```

## Object Detection Demo

See [README_objdet.md](./README_objdet.md).

## Launch File Parameters

Parameter                | Description                                                                | Value
-------------------------|----------------------------------------------------------------------------|-------------------
rosparam file            | Algorithm configuration parameters (see "ROSPARAM Parameters" section)     | config/params.yaml
image_format             | Input image format: 0 - VX_DF_IMAGE_U8, 1 - VX_DF_IMAGE_NV12, 2 - VX_DF_IMAGE_UYVY | 0, 1, 2
enable_ldc_node          | LDC node enable/disable flag                                               | 0, 1
lut_file_path            | LDC rectification table path                                               | String
dl_model_path            | DL model artifacts folder path                                             | String
input_topic_name         | Subscribe topic name for input camera image                                | camera/right/image_raw
rectified_image_topic    | Publish topic name for output rectified image                              | camera/right/image_rect_nv12
rectified_image_frame_id | frame_id for output rectified image                                        | right_frame
vision_cnn_tensor_topic  | Publish topic name for output tensor                                       | vision_cnn/tensor
exportPerfStats          | Flag for exporting the performance data to a file: 0 - disable, 1 - enable | 0, 1

## ROSPARAM Parameters
The table below describes the parameters in `config/params.yaml`:

 Parameter                | Description                                        | Value
--------------------------|----------------------------------------------------|----------
 width                    | Input image width                                  | Integer
 height                   | Input image height                                 | Integer
 num_classes              | Number of semantic segmentation classes            | 0 - 20
 pipeline_depth           | OpenVX graph pipeline depth                        | 1 - 4
 log_level                | Log level for the app                              | 0 - 4
 ## Processing Blocks

Referring to Figure 1, below are the descriptions of the processing blocks implemented in this application:

1. The first step to process input images with lens distortion, TDA4 LDC (Lens Distortion Correction) hardware accelerator (HWA) does un-distortion or rectification. Pseudo codes to create LDC tables for rectification are provided [here](../ti_sde/README.md). Note that the LDC HWA not only removes lens distortion or rectifies, but also changes image format. Input image to the application is of YUV422 (UYVY) format, which is converted to YUV420 (NV12) by the LDC.
2. Input images are resized to match to the input tensor size of the semantic segmentation model. The MSC (Multi-Scaler) HWA resizes the images to a desired size.
3. The pre-processing block, which runs on C6x, converts YUV420 (NV12) to RGB, which is expected input format for the semantic segmentation network.
4. The semantic segmentation network is accelerated by C7x/MMA with DLR runtime, and outputs a tensor that has class information for every pixel.

## Known Issue

1. The default semantic segmentation CNN was trained with Cityscapes dataset first, and then re-trained with a small dataset collected from a stereo camera (ZED camera, HD mode) for a limited scenarios with coarse annotation. Therefore, the model can show limited accuracy performance if a different camera model is used and/or when it is applied in different environmental scenes.
