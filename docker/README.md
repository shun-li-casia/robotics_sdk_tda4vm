
Setting Up Robotics SDK
=======================

This section describes how to set up the Robotics SDK on the TDA4 Processor SDK Linux.

## 1. Requirements & Dependency

### 1.1. Supported Hardware Platforms

 Platform | Supported Devices                           | Supported EVMs
----------|---------------------------------------------|-----------------------------------------------
 J721E    | [TDA4VM](https://www.ti.com/product/TDA4VM) | [SK-TDA4VM](https://www.ti.com/tool/SK-TDA4VM)

### 1.2. Processor SDK Linux for Edge AI
The Robotics SDK requires [the SD card image](http://udc0393891.dhcp.ti.com/webgen/publish/nightly/PROCESSOR_SDK_LINUX_SK_TDA4VM/08_02_00_09/exports/ti-processor-sdk-linux-sk-tda4vm-etcher-image.zip) [TODO: update URL] from [Processor SDK Linux for Edge AI 8.2.0](https://www.ti.com/tool/download/PROCESSOR-SDK-LINUX-SK-TDA4VM#release-notes/08.02.00.01) [TODO: update URL]. The SD card image contains Processor SDK Linux and libraries that are necessary for setting up the Robotics SDK environment.

### 1.3. Ubuntu PC
A Ubuntu PC is required for visualization of ROS topics published from the TDA4 target. We have tested only with native x86_64 Ubuntu PCs, and have **not** tested with any other Ubuntu systems: including Ubuntu virtual machines and Docker Desktop on Mac or Windows.

It is assumed that matching ROS distro (either ROS 1 Noetic or ROS 2 Foxy) is installed on the remote Ubuntu PC, either in the host Ubuntu filesystem natively or in a Docker container. In case when you want to install ROS natively on the remote Ubuntu filesystem:
* For ROS Noetic installation steps, please refer to [this ROS wiki page](http://wiki.ros.org/noetic/Installation/Ubuntu).
* For ROS 2 Foxy installation steps, please refer to [this ROS 2 documentation](https://docs.ros.org/en/foxy/Installation.html).

We also provide Dockerfiles (`docker/Dockerfile.x86_64.{noetic,foxy}`) that can build and run on the remote Ubuntu PC for both ROS Noetic and ROX 2 Foxy with detailed steps for setting up Docker environment on the remote PC. For installation of Docker on the Ubuntu PC, the following links may be useful:
* [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
* [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

### 1.4. USB Camera
The Robotics SDK provides camera ROS nodes for [ZED stereo camera](https://www.stereolabs.com/zed/), USB mono cameras (Logitech C270, C920, C922), and CSI camera ([OV5640](https://www.leopardimaging.com/product/cmos-sensor-modules/mipi-camera-modules/li-am65x-csi2)). All the demo applications can be tried out with a live camera as well as with a ROSBAG file that is provided with the SDK.

* For configuration of a stereo camera, please see [ros1/drivers/zed_capture/README](../ros1/drivers/zed_capture/README.md).
* For configuration of a USB mono camera, please see [ros1/drivers/mono_capture/README](../ros1/drivers/mono_capture/README.md).
* For GStreamer camera ROS node (USB cameras are also supported), please see [ros1/drivers/gscam/README_TI](../ros1/drivers/gscam/README_TI.md) and [ros2/drivers/gscam2/README_TI](../ros2/drivers/gscam2/README_TI.md)


**NOTE**: For more stable connection, it is recommended to connect the ZED camera to the USB Type-C port of the TDA4 Board with a USB type-A to type-C adaptor.

![](docs/tiovx_ros_setup.svg)
<figcaption>Figure 1. Robotics SDK Setup and Installation Steps</figcaption>
<br />

<!-- ================================================================================= -->
## 2. Set Up the TDA4 Target and Development Environment
Figure 1 shows the hardware setup and high-level installation steps on the TDA4 target and the Ubuntu PC. The target EVM and the remote Ubuntu PC are assumed to be connected through Ethernet in the same network.

### 2.1. Build SD Card

1. From Ubuntu PC, download [the SD card image](http://udc0393891.dhcp.ti.com/webgen/publish/nightly/PROCESSOR_SDK_LINUX_SK_TDA4VM/08_02_00_09/exports/ti-processor-sdk-linux-sk-tda4vm-etcher-image.zip) [TODO: update URL].

2. Flash the downloaded image to a SD card (minimum 32GB, high-performance) using Balena Etcher tool. For detailed instruction, please refer to [this section](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_01_00_02/exports/docs/getting_started.html#software-setup) [TODO: update URL].

**NOTE**: The etcher image is created for 16 GB SD cards, if you are using a larger SD card, it is highly recommended to expand the root filesystem to use the full SD card capacity using below steps on the Ubuntu PC.

```
# find the SD card device entry using lsblk (Eg: /dev/sdb)
# use the following commands to expand the filesystem

umount /dev/sdX1
umount /dev/sdX2
sudo parted -s /dev/sdX resizepart 2 '100%'
sudo e2fsck -f /dev/sdX2
sudo resize2fs /dev/sdX2

# replace /dev/sdX in above commands with SD card device entry
```

### 2.2. Connect Remotely to the TDA4 Target

1. To find the IP address assigned to the EVM, use a serial port communications program (for example, `sudo minicom -D /dev/ttyUSBx` where `/dev/ttyUSBx` is the device for the UART serial port), log in with `root` account, and run `ifconfig`.

2. From a terminal on the PC, open a SSH session to connect remotely to the TDA4 target:
    ```
    user@pc:~$ ssh root@<TDA4_IP_address>
    ```
    **Note**: It is recommended to use a *static* IP for the TDA4 EVM to make ROS network setting easy.<br />
    You can consider using VS Code with "remote development extension pack" for better experience, in a similar way as described in [this section of Edge AI documentation](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_01_00_02/exports/docs/getting_started.html#connect-remotely).

### 2.3. Initial Setup to Use the Robotics SDK
### 2.3.1 On the TDA4 Target
You can run the installation script on the TDA4 target as follows:
```
root@tda4vm-sk:~$ cd /opt/robotics_sdk
root@tda4vm-sk:~$ source ./install_robotics_sdk.sh
```
This script takes care of:
* Cloning the main GIT repository for Robotics SDK under `/opt/robotics_sdk`
* Setting up the folders for evaluating the Robotics SDK under `$HOME/j7ros_home`
* Downloading ROSBAG and other data files
* Downloading several deep-learning models from the edge AI model zoo. You can also use the model downloader tool (please refer to [this section](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_01_00_02/exports/docs/inference_models.html) [TODO: update URL] for more details).

### 2.3.2 On the Remote Ubuntu PC
In a similar way, you can use the same script to set up on the remote Ubuntu PC for visualization:
```
user@pc:~$ wget -O init_setup.sh https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/plain/init_setup.sh
user@pc:~$ source ./init_setup.sh REL.08.02.00
```

**NOTE**: In a proxy network, in case the `wget` command above does not work, you can try again with adding `--proxy off` argument:
```
user@pc:~$ wget --proxy off -O init_setup.sh https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/plain/init_setup.sh
```

### 2.4. Set Up Docker Environment on the TDA4 Target

The Robotics SDK runs in a Docker container environment on the Processor SDK Linux for Edge AI. In the Robotics SDK Docker environment, ROS and necessary libraries and tools are installed.

First, following [this link](https://docs.docker.com/get-started/#test-docker-installation), please check that Docker and network work correctly on the TDA4 target.

The following two sections describe the Docker environment setup, details of building, and running apps under Docker, for ROS 1 and ROS 2, respectively. Please note that it will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`.

#### [Docker Setup for ROS 1 Noetic](./setting_docker_ros1.md)

#### [Docker Setup for ROS 2 Foxy](./setting_docker_ros2.md)

---
**NOTE**

**Proxy Network**: If the board running the Docker container is behind a proxy server, the default settings for downloading files and installing packages via `apt-get` may not work. If you are running the board from TI network, docker_build and docker_run scripts will automatically detect and configure necessary proxy settings. For other cases, you may need to modify the configuration files under `docker/proxy` folder and to add the custom proxy settings required for your network, and relevant port of `docker_build_rosX.sh` and `docker_run_rosX.sh`

**Docker in TI Network (only for TI Users)**: For proxy settings in TI network, you can check [this page](https://confluence.itg.ti.com/display/J7TDA4xSW/Docker+on+PC+in+TI+Proxy+Network).

**Docker Start**: After "docker build" is completed, it is important to use `docker_run_rosX.sh` script to start a Docker container, since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the TDA4 device. Please note that `docker_run_rosX.sh` includes `--rm` argument by default. Just remove `--rm` argument in `docker_run_rosX.sh` in case you want to do "docker commit" after exiting a Docker container. A short information about several useful Docker commands is provided in [this link](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_01_00_02/exports/docs/docker_environment.html#additional-docker-commands).

**Switching between ROS 1 and ROS 2 Docker Containers**: When the applications are built under the ROS1 container, two directories, `{build, devel}` are created under **ros_ws** directory under TDA4 host and similarly `{build, install, log}` directories are created when applications are built under ROS 2 container. Since the containers share the common space `ros_ws` on TDA4 host Linux filesystem, these directories need to be removed if switching between ROS 1 and ROS 2 containers. Alternatively, we can create different folders for each of ROS distro and apply soft-links as shown below as example before running "docker run" script.

```
# Before launching ROS 1 container, establish the following soft-links:
$ROS_WS/build -> $ROS_WS/build_noetic
$ROS_WS/devel -> $ROS_WS/devel_noetic

# Before launching ROS 2 container, establish the following soft-links:
$ROS_WS/build -> $ROS_WS/build_foxy
$ROS_WS/install -> $ROS_WS/devel_foxy
$ROS_WS/log -> $ROS_WS/log_foxy
```
