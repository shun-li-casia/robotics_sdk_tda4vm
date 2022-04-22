Change Log
==========

## 8.2.0 (2022-04-22)

* ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.2.0 pre-built image.
* A new 3D perception demo application (`ti_objdet_range`) that is based on CNN object detection and stereo vision is added.
* OpenCV based stereo camera calibration tool is added. Using checkerboard chart images, the Python tool calibrates stereo cameras to update camera parameters and stereo rectification tables.
* Moved the mono and stereo camera tools under `tools` folder.
* Deep-learning runtime Python packages for ONNX, TFLite, TVM-DLR are added in the Robotics SDK ROS Docker containers.
* Radar driver ROS node for TI mmWave radar devices (including IWR6843ISK) is added. The ROS node configures the mmWave device and publishes a ROS PointCloud2 message for the objects detected.
* The default semantic segmentation models are now part of Edge AI model zoo: TVM-SS-5818-deeplabv3lite-mobv2-qat-robokit-768x432, ONR-SS-8818-deeplabv3lite-mobv2-qat-robokit-768x432.
* The default 2D object detection model (ONR-OD-8080-yolov3-lite-regNetX-1.6gf-bgr-mmdet-coco-512x512) is improved. Initial loading time is significantly reduced.
* Added support for WiFi station mode with Intel Wireless-AC 9260 M.2 WiFi module (as part of Processor SDK Linux for Edge AI).
* Hardware PWM is enabled (as part of Processor SDK Linux for Edge AI).

## 8.1.0 (2022-02-04)

* Version labeling for the Robotics SDK starts following the version numbers of the Processor SDK Linux releases.
* ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.1.0 pre-built image.
* Migrated to ROS Noetic: Most of demo applications run in ROS Noetic as well as in ROS 2 Foxy.
* Added GStreamer-based camera ROS nodes: Open-source 'gscam' (for ROS 1 Noetic) and 'gscam2' (in ROS 2 Foxy) packages are customized to better integrate with TDA4-optimized processing chains, also leveraging the Edge AI GStreamer plugins optimized on TDA4.
* Added CSI camera support: OV5640 CSI camera is supported with the GStreamer-based camera ROS nodes.
* Performance benchmarking report is provided in the documentation.
* Added GIPO libraries for Python and C++ (as part of Processor SDK Linux for Edge AI).
* SDK installation path is moved to /opt/robotics_sdk on the TDA4 root filesystem.

## 0.5.0 (2021-09-28)

* ROS development environment in a Docker container on top of Processor Linux SDK for edge AI 8.0.1 pre-built image.
* A new versatile vision CNN inference ROS node is added. This new CNN inference node supports many 2D object detection, and semantic segmentation models from the TI edge AI model zoo, that run in any of ONNX-RT, TFL-RT, and Neo-AI-DLR runtime frameworks.
* Added ROS 2 support: Docker environment for ROS 2 Foxy is provided. Most of demo applications run in ROS 2 Foxy as well as in ROS 1 Melodic.
* A visual localization demo application where DKAZE feature extraction is accelerated on C7/MMA.
* USB mono camera support is added: we provide a OpenCV based ROS driver for capturing raw images from a USB camera (e.g., Logitech C920, C922, C270).
* Camera calibration tool and LDC remap look-up-table generation tool for USB mono cameras are also provided.
* Open-source 2D Lidar SLAM: we provide a script and instruction for evaluating open-source Hector SLAM on TDA4 device.
* Dockerfiles for the remote visualization PC are provided for each of ROS 1 Melodic and ROS 2 Foxy.

## 0.4.0 (2021-07-14)

* ROS development environment in a Docker container on top of TI Edge AI 0.5 base image
* Proxy settings are provided for building and running the Docker image behind a proxy network (TI network as an example)

## 0.3.0 (2021-04-15)

* ROS development environment in a Docker container on top of Processor SDK 7.3.0 pre-built image
* Enhanced the stereo vision demo application: added point-cloud generation
* Updated the semantic segmentation demo application: migrated to open-source deep-learning runtime (TVM + Neo-AI-DLR)
* Added a new demo application: 3D obstacle detection accelerated on deep-learning core (C7/MMA) and hardware accelerators (SDE, LDC, MSC)
* USB stereo camera ROS driver node for ZED cameras
* Stereo rectification LDC lookup-table generation tool for ZED cameras
* A live USB stereo camera support for all three demo applications (stereo vision, semantic segmentation, and 3D obstacle detection)

## 0.1.0 (2020-12-15)

* Released with Processor SDK RTOS 7.1.0
* TI OpenVX (TIOVX) with ROS development framework
* TI Vision Apps Library deployed on the J721e target that enables building applications directly on the target
* Docker container environment on J721e for TIOVX + ROS development framework
* Demo application: stereo vision processing node accelerated on LDC and SDE
* Demo application: CNN semantic segmentation node with TIDL running on C7x/MMA