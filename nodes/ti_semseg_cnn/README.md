Semantic Segmentation Application on ROS
========================================

It demonstrates the semantic segmentation application using a TIDL (TI Deep Learning) CNN network running on J7 C7x/MMA. However, as shown in Figure 1, this application consists of multiple processing blocks that consumes other J7 HWAs and processors besides C7x/MMA.

<figure class="image">
  <center><img src="./docs/ti_semseg_cnn_data_flow.png"></center>
  <figcaption> <center>Figure 1. Overall semantic segmentation application flow </center></figcaption>
</figure>

## Semantic Segmentation CNN with TI Deep-Learning (TIDL) Development Tool
A CNN model for semantic segmentation has been developed using [Jacinto AI DevKit (PyTorch)](https://git.ti.com/cgit/jacinto-ai/pytorch-jacinto-ai-devkit/about/). 


* Model: deeplabv3lite_mobilenetv2_tv (for details, see [LINK](https://git.ti.com/cgit/jacinto-ai/pytorch-jacinto-ai-devkit/about/docs/Semantic_Segmentation.md))
* Input image: 768 x 432 pixels in RGB
* Training data: [Cityscapes Dataset](https://www.cityscapes-dataset.com)

## Node Description

`ti_semseg_cnn` directory structure is shown below:
```
ti_semseg_cnn
.
├── CMakeLists.txt
├── config
│   └── params.yaml
├── docs
│   └── ti_semseg_cnn_data_flow.png
├── launch
│   ├── bag_semseg_cnn.launch
│   └── semseg_cnn.launch
├── package.xml
├── README.md
└── src
    ├── semseg_cnn.cpp
    ├── semseg_cnn.h
    ├── semseg_cnn_main.cpp
    ├── semseg_cnn_node.cpp
    └── semseg_cnn_node.h
```

This application can be run by launching semseg_cnn.launch file, i.e.,
```
roslaunch ti_semseg_cnn semseg_cnn.launch
```
It is recommended to launch bag_semseg_cnn.launch file if a ROSBAG file needs to be played as well.

`semseg_cnn.launch` file specifies the followings:

* YAML file that includes algorithm configuration parameters. For the descriptions of important parameters, refer to Parameter section below. For the description of all parameters, please see a yaml file.
* Input topic name to read input images.
* Output undistorted or rectified image topic name.
* Output semantic segmentation image topic name when an color-coded semantic segmentation map is published.
* Flag that indicates the color-coded semantic segmentation map is published in RGB format. If this flag is false, it is published in YUV420 format.
* Output semantic segmentation tensor topic name when the output tensor is published.

When the semantic segmentation output tensor is published by the application, the color-coded semantic segmentation can be created alternatively by launching the `ti_viz_nodes` application, i.e.,
```
roslaunch ti_viz_nodes viz_semseg.launch
```

## Parameters

 Parameter                | Description                                                                  | Value
--------------------------|------------------------------------------------------------------------------|----------
 lut_file_path            | LDC rectification table path                                                 | String
 tidl_config_file_path    | TIDL config file path                                                        | String
 tidl_network_file_path   | TIDL network file path                                                       | String
 tidl_network_file_path   | TIDL network file path                                                       | String
 width                    | Input image width                                                            | Integer
 height                   | Input image height                                                           | Integer
 dl_width                 | Image width to TIDL network                                                  | Integer
 dl_height                | Image height to TIDL network                                                 | Integer
 out_width                | Output semantic segmentation output (tensor) width                           | Integer
 out_height               | Output semantic segmentation output (tensor) height                          | Integer
 num_classes              | Number of semantic segmentation classes                                      | 0 ~ 20
 enable_post_proc         | Flag to indicate if the post processing of th TIDL output should be enabled  | 0, 1
 pipeline_depth           | OpenVX graph pipeline depth                                                  | 1 ~ 4

## Processing Blocks

Please refer to Figure 1 for the following descriptions of the processing blocks implemented for this application. 

1. When input images are distorted or unrectified, they are undistorted or rectified by the J7 LDC (Lens Distortion Correction) HWA. Pseudo codes to create LDC tables for rectification are described [here](../ti_sde/README.md). Note that the LDC HWA not only removes lens distortion or rectifies, but also changes image format. Input image to the application is of YUV422 (UYVY) format, and YUV422 input is converted to YUV420 (NV12) by LDC.
2. Input images are resized to a smaller resolution, which is specified by `dl_width` and `dl_height` in `params.yaml`, for the TIDL semantic segmentation network. The MSC (Multi-Scaler) HWA is used to resize input images.
3. The pre-processing block, which runs on C6x, converts YUV420 to RGB, so that the TIDL semantic segmentation network can read input images.
4. The TIDL semantic segmentation network is accelerated by C7x/MMA and outputs a tensor that has class information for every pixel.
5. The post-processing block, which runs on C6x, creates a color-coded semantic segmentation map image from the output tensor. It can be enabled or disabled by configuring `enable_post_proc` parameter in `params.yaml`. Only if the post-processing block is enabled, the color-coded semantic segmentation map is created and published. Its format is YUV420. When `output_rgb` is true in the launch file, it is published in RGB format after conversion. If the post-processing, the semantic segmentation output tensor from the TIDL network is published instead.

## Known Issue

1. The `ti_semseg_cnn` semantic segmentation CNN was trained with Cityscapes dataset first, and then re-trained with a small dataset collected from a stereo camera (ZED-1 camera, HD mode) with coarse annotation. Therefore, accuracy performance of the CNN model can be improved with a larger dataset and fine annotation.

