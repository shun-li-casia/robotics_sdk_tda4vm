Robotics Software Development Kit
=================================

## Introduction to TI OpenVX + ROS Development Framework

<figure class="image">
    <center><img src="docker/docs/tiovx_ros_sw_stack.png" style="width:726px; height:398px;"/></center>
    <figcaption> <center>Figure 1. TI OpenVX + ROS Framework: Software Stack </center></figcaption>
</figure>

The TI OpenVX + ROS development framework runs in a Docker container environment on J7 Processor SDK Linux. We provide detailed steps for setting a Docker container environment for ROS Melodic along with the TI Vision Apps Library (see next section). The TI OpenVX + ROS development framework allows:

* Optimized software implementation of computation-intensive software blocks (including deep-learning, vision, perception, and ADAS) on deep-learning core (C7x/MMA), DSP cores, hardware accelerators built-in on the Jacinto 7 processor
* Application softwares can be complied directly on the Jacinto 7 target using APIs optimized on the Jacinto 7 cores and hardware accelerators along with many open-source libraries and packages including, for example, OpenCV and Point-Cloud Library (PCL).

Figure below is a representative vision application developed in TI OpenVX + ROS framework.

<figure class="image">
    <center><img src="docker/docs/tiovx_ros_demo_diagram.svg" style="width:896px;"/></center>
    <figcaption> <center>Figure 2. Example Application in TI OpenVX + ROS Framework </center></figcaption>
</figure>

### TI Vision Apps Library
The TI Vision Apps Library is a set of APIs for the target deployment that are derived from the Jacinto 7 Processor SDK RTOS which includes:

* TI OpenVX kernels and infrastructure
* TI deep-learning (TIDL) applications
* Imaging and vision applications
* Advanced driver-assistance systems (ADAS) applications
* Perception applications

The TI Vision Apps Library is included in the pre-built package of [J721E Processor SDK RTOS 7.3.0](https://www.ti.com/tool/download/PROCESSOR-SDK-RTOS-J721E/07.03.00.07).

### Open-Source Deep-Learning Runtime
The J721E Processor SDK RTOS 7.3.0 also supports the following open-source deep-learning runtime:
* TVM/Neo-AI-DLR
* TFLite Runtime
* ONNX Runtime

We provides two demo applications that include a deep-learning model that is implemented on TVM/Neo-AI-DLR runtime library.

## Setting Up Robotics SDK Docker Container Environment on J7 Target
<a href="https://software-dl.ti.com/jacinto7/esd/processor-sdk-rtos-jacinto7/ros_perception/j7ros_docker_readme_00_03_00.pdf" download>Click to Download "j7ros_docker_readme.pdf"</a>

For debugging: [docker/README.md](docker/README.md) (Caution: there are issues in rendering markdown files)

## Demo Applications

<figure class="image">
    <center><img src="nodes/ti_estop/docs/estop_rviz.png" style="width:700px;"/></center>
    <figcaption> <center>Figure 3. Demo Applications </center></figcaption>
</figure>

### [Stereo Vision Processing Accelerated on LDC and SDE](nodes/ti_sde/README.md)

### [Semantic Segmentation Accelerated on C7x/MMA](nodes/ti_semseg_cnn/README.md)

### [3D Obstacle Detection Accelerated on SDE and C7x/MMA](nodes/ti_estop/README.md)

### [USB Stereo Camera Capture Node for ZED Cameras](drivers/zed_capture/README.md)
## Change Log
See [CHANGELOG.md](CHANGELOG.md)
## Limitations and Known Issues

1. RViz visualization is displayed on a remote Ubuntu PC. Display from insider a Docker container on the J7 target is not enabled and tested.
2. Ctrl+C termination of a ROS node or a ROS launch session can be sometimes slow. 
3. Stereo Vision Demo
    * Output disparity map may have artifacts that are common to block-based stereo algorithms. e.g., noise in the sky, texture-less area, repeated patterns, etc.
    * While the confidence map from SDE has 8 values between 0 (least confident) to 7 (most confident), the confidence map from the multi-layer SDE refinement has only 2 values, 0 and 7. Therefore, it would not appear as fine as the SDE's confidence map.
4. The semantic segmentation model used in `ti_semseg_cnn` and `ti_estop` nodes was trained with Cityscapes dataset first, and  re-trained with a small dataset collected from a particular stereo camera (ZED camera, HD mode) for a limited scenarios with coarse annotation. Therefore, the model can show limited accuracy performance if a different camera model is used and/or when it is applied in different environment scenes.

## Questions & Feedback

If you have questions or feedback, please use [TI E2E](https://e2e.ti.com/support/processors).
