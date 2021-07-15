Robotics Software Development Kit
=================================
## Git Repository

[Robotics SDK Git Repository](https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/about/)

[User Guide Documentation](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/00_04_00_01/docs/index.html)
## TI Robotics Development Framework

![](docker/docs/tiovx_ros_sw_stack.png)
 <figcaption>Figure 1. TI Robotics Software Development Kit: Software Stack </figcaption>
 <br />

The TI Robotics Software Development Kit (SDK) runs in a Docker container environment on J7 Processor SDK Linux provided with TI Edge AI base image. We provide detailed steps for setting a Docker container environment for ROS Melodic along with the TI Vision Apps Library (see next section). The Robotics SDK allows:

* Optimized software implementation of computation-intensive software blocks (including deep-learning, vision, perception, and ADAS) on deep-learning core (C7x/MMA), DSP cores, hardware accelerators built-in on the Jacinto 7 processors.
* Application software can be complied directly on the Jacinto 7 target using APIs optimized on the Jacinto 7 cores and hardware accelerators along with many open-source libraries and packages including, for example, OpenCV, Point-Cloud Library (PCL), and many more.

Figure below is a representative deep-learning and compute-intensive application developed with the TI Robotics SDK.

![](docker/docs/tiovx_ros_demo_diagram.svg)
<figcaption>Figure 2. Example Application with TI Robotics SDK </figcaption>
<br />

### TI Vision Apps Library
TI Vision Apps Library is a set of APIs for the target deployment that are derived from the Jacinto 7 Processor SDK RTOS which includes:

* TI OpenVX kernels and infrastructure
* TI deep-learning (TIDL) applications
* Imaging and vision applications
* Advanced driver-assistance systems (ADAS) applications
* Perception applications

The TI Vision Apps Library is included in the pre-built base image for  [TI Edge AI Development Kit 0.5](https://software-dl.ti.com/jacinto7/esd/edgeai-devkit/00_05_00_02/exports/docs/index.html).

### Open-Source Deep-Learning Runtime
The Edge AI Development Kit 0.5 also supports the following open-source deep-learning runtime:
* TVM/Neo-AI-DLR
* TFLite Runtime
* ONNX Runtime

For more details on open-source deep-learning runtime on J7/TDA4x, please check [TI Edge AI Cloud](https://dev.ti.com/edgeai/). We provides two demo applications that include a deep-learning model that is implemented in the TVM/Neo-AI-DLR framework.

## Setting Up Robotics SDK Docker Container Environment

See [Setting Up Robotics SDK Environment](docker/README.md)

**Note**: git.ti.com has some issue in rendering markdown files. We highly recommend to use [the section in the User Guide Documentation](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/00_04_00_01/docs/source/docker/README.html#setting-up-robotics-kit-environment)

## Sensor Driver Nodes
### [USB Stereo Camera Capture Node for ZED Cameras](drivers/zed_capture/README.md)

## Demo Applications

![](nodes/ti_estop/docs/estop_rviz.png)
<figcaption>Figure 3. Demo Applications </figcaption>
<br />

### [Stereo Vision Processing Accelerated on LDC and SDE](nodes/ti_sde/README.md)

### [Semantic Segmentation Accelerated on C7x/MMA](nodes/ti_semseg_cnn/README.md)

### [3D Obstacle Detection Accelerated on SDE and C7x/MMA](nodes/ti_estop/README.md)

## Change Log
See [CHANGELOG.md](CHANGELOG.md)
## Limitations and Known Issues

1. RViz visualization is displayed on a remote Ubuntu PC.
2. Ctrl+C termination of a ROS node or a ROS launch session can be sometimes slow.
3. Stereo Vision Demo
    * Output disparity map may have artifacts that are common to block-based stereo algorithms. e.g., noise in the sky, texture-less area, repeated patterns, etc.
    * While the confidence map from SDE has 8 values between 0 (least confident) to 7 (most confident), the confidence map from the multi-layer SDE refinement has only 2 values, 0 and 7. Therefore, it would not appear as fine as the SDE's confidence map.
4. The semantic segmentation model used in `ti_semseg_cnn` and `ti_estop` nodes was trained with Cityscapes dataset first, and  re-trained with a small dataset collected from a particular stereo camera (ZED camera, HD mode) for a limited scenarios with coarse annotation. Therefore, the model can show limited accuracy performance if a different camera model is used and/or when it is applied to different environment scenes.

## Questions & Feedback

If you have questions or feedback, please use [TI E2E](https://e2e.ti.com/support/processors).
