Change Log
==========

## 0.3.0 (2021-04-15)

* ROS development environment in a Docker container op top of Processor SDK 7.3.0 pre-built image
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