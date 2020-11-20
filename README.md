TI OpenVX + ROS Framework & Applications
========================================

### Introduction to TI OpenVX + ROS Development Framework

<figure class="image">
    <center><img src="docker/docs/tiovx_ros_sw_stack.png"/></center>
    <figcaption> <center>Figure 1. TI OpenVX + ROS Framework: Software Stack </center></figcaption>
</figure>

The TI OpenVX + ROS development framework is enabled in a Docker container environment on J7 Processor SDK Linux. We provide detailed steps for setting a Docker container environment for ROS Melodic together with the TI Vision Apps Library (see next section). The TI OpenVX + ROS development framework allows:

- Optimized software implementation of computation-intensive software blocks (including deep-learning, vision, perception, and ADAS) on deep-learning core (C7x/MMA), DSP cores, hardware accelerators built-in on the Jacinto 7 processor
- Application softwares can be complied directly on the Jacinto 7 processor in a Docker container using APIs optimized on Jacinto 7 processor along with many open-source libraries and packages including, for example. OpenCV and Point-Cloud Library (PCL).

Figure below is a representative vision application that can be developed in TI OpenVX + ROS framework.

<figure class="image">
    <center><img src="docker/docs/tiovx_ros_demo_diagram.png" style="width: 800px;"/></center>
    <figcaption> <center>Figure 2. Example Application in TI OpenVX + ROS Framework </center></figcaption>
</figure>

### TI Vision Apps Library
The TI Vision Apps Library is a set of APIs for the target deployment that are derived from the Jacinto 7 Processor SDK RTOS, which includes:
- TI OpenVX kernels and infrastructure
- TI deep learning (TIDL) applications
- Imaging and vision applications
- Advanced driver-assistance systems (ADAS) applications
- Perception applications

## How to Set Up TI OpenVX + ROS Docker Container Environment on J7 Target
See [LINK](docker/README.md)

## TI OpenVX + ROS Demo Applications

### [Stereo Vision Processing Node Accelerated on LDC and SDE](nodes/ti_sde/README.md)

### [CNN Semantic Segmentation Node with TIDL Running on C7x/MMA](nodes/ti_semseg_cnn/README.md)

## Known Issues

1. Display is not enabled from a Docker container on J7.
2. The inference-time accuracy of `ti_semseg_cnn` semantic segmentation CNN is not great. This is expected since the CNN network was trained with Cityscapes dataset, and was not re-trained to further optimize the CNN model on the camera data that played back from the ROSBAG file.
3. Ctrl+C terminal of a ROS node or a ROS launch session may be slow and sometimes does not  fully clean up OpenVX part of implementation, causing VX_ERROR in the next launching. When this happens, it is recommended to reboot the J7 EVM.