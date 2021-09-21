
Setting Up Robotics SDK
=======================

## 1. Requirements & Dependency

### 1.1. Supported Hardware Platforms

 Platform | Supported Devices                           | Supported EVM
----------|---------------------------------------------|-----------------------------------------------
 J721E    | [TDA4VM](https://www.ti.com/product/TDA4VM) | [SK-TDA4VM](https://www.ti.com/tool/SK-TDA4VM), [TDA4VMXEVM](https://www.ti.com/tool/TDA4VMXEVM)

### 1.2. Processor SDK Linux for Edge AI
The Robotics SDK requires [the SD card image](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/ti-processor-sdk-linux-sk-tda4vm-etcher-image.zip) from [Processor SDK Linux for Edge AI](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/docs/index.html). The SD card image contains Processor SDK Linux and libraries that are necessary for setting up the Robotics SDK environment.

### 1.3. Ubuntu PC
A Ubuntu PC is required for visualization of ROS topics published from the TDA4 target. It is assumed that matching ROS distro (either Melodic or ROS 2 Foxy) is installed on the remote Ubuntu PC or in a Docker container.

In case of directly installation of ROS on the Ubuntu PC:
* For ROS Melodic installation steps, please refer to [this ROS wiki page](http://wiki.ros.org/melodic/Installation/Ubuntu).
* For ROS 2 Foxy installation steps, please refer to [this ROS 2 documentation](https://docs.ros.org/en/foxy/Installation.html).

We also provide Dockefiles that can build and run on the remote Ubuntu PC for both ROS Medlodic and ROX 2 Foxy with detailed steps for setting up Docker environment on the remote PC.

### 1.4. USB Camera [Optional]
The Robotics SDK provides OpenCV-based ROS drivers for [ZED stereo camera](https://www.stereolabs.com/zed/) and USB mono cameras (Logitech C270, C920, C922). All the demo applications can be tried out with a live camera as well as a ROSBAG file that is provided.

* For configuration of a stereo camera, please see `{ros1,ros2}/drivers/zed_capture/README.md`.
* For configuration of a USB mono camera, please see `{ros1,ros2}/drivers/mono_capture/README.md`.

![](docs/tiovx_ros_setup.svg)
<figcaption>Figure 1. Robotics SDK Setup and Installation Steps</figcaption>
<br />

<!-- ================================================================================= -->
## 2. Set Up the TDA4 Target and Development Environment
Figure 1 shows the hardware setup and high-level installation steps on the TDA4 target and the Ubuntu PC.

### 2.1. Build SD Card

1. From Ubuntu PC, download [the SD card image](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/ti-processor-sdk-linux-sk-tda4vm-etcher-image.zip).

2. Flash the downloaded image to a SD card (minimum 32GB, high-performance) using Balena etcher tool. For detailed instruction, please refer to [this section](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/docs/getting_started.html#software-setup).

### 2.2. Connect Remotely to the TDA4 Target

1. To find the IP address assigned to the EVM, use a serial port communications program (for example, `sudo minicom -D /dev/ttyUSBx` where `/dev/ttyUSBx` is the device for the UART serial port), log in with `root` account, and run `ifconfig`.

2. From a terminal on the PC, open a SSH session to connect remotely to the J7 target:
    ```
    user@pc:~$ ssh root@<J7_IP_address>
    ```
    **Note**: It is recommended to use a *static* IP for the J7 EVM to make ROS network setting easy.<br />
    You can consider using VS Code with "remote development extension pack" for better experience, in a similar way as described in [this section of Edge AI documentation](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/docs/getting_started.html#connect-remotely)

### 2.3. Clone the Robotics SDK Git Repository and Intial Setup
You can download `initial_setup.sh` and run the the script:
```
root@j7-evm:~$ wget https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/tree/initial_setup.sh
root@j7-evm:~$ source initial_setup.sh
```
This scipt takes care of:
* Setting up the folders for evaluating the Robotics SDK under $HOME/j7ros_home
* Downloadding DL runtime libraries, ROSBAG and other data files
* Downloadding several DL models in the edge AI model zoo

<!-- **NOTE**: Subsections below are just for reference. Equivalent settings are already included in `initial_setup.sh`.

### 2.3.1. Setting Up Edge AI

1. Installing dependencies for Edge AI:
    ```
    root@j7-evm:/opt/edge_ai_apps$ ./setup_script.sh
    ```
    This script takes care of installing header files for deep-learning runtime libraries among others.

2. Download Model Zoo Deep-Learnign Models for Edge Inference:
    Below command is for downloading all object detection and semantic segmentaiton models avaiable in the model zoo:
    ```
    root@j7-evm:/opt/edge_ai_apps$ ./download_models.sh -d detection segmentation
    ```
    For more information about the model downloader tool, please refer to [this section](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/docs/inference_models.html).

### 2.3.2. Clone the Robotics SDK Git Repository

1. Set up the project directory and the ROS workspace:
    ```
    root@j7-evm:~$ WORK_DIR=$HOME/j7ros_home
    root@j7-evm:~$ ROS_WS=$WORK_DIR/ros_ws
    root@j7-evm:~$ mkdir -p $ROS_WS/src
    root@j7-evm:~$ cd $ROS_WS/src
    ```
2. Clone the Robotics SDK GIT repository:
    ```
    git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git
    ```

### 2.3.3. Download ROSBAG and Data Files

1. For convenience, set up a soft-link:
    ```
    root@j7-evm:~/j7ros_home/ros_ws/src$ cd $WORK_DIR
    root@j7-evm:~/j7ros_home$ ln -sf $ROS_WS/src/jacinto_ros_perception/docker/Makefile
    ```
2. To download and install ROSBAG and other files, run the following in `$WORK_DIR`:
    ```
    root@j7-evm:~/j7ros_home$ make data_download
    ```
    This script downloads several tarballs and installs them under `$WORK_DIR/data` and `\opt\dl_runtime`. -->

### 2.4. Set Up Docker Environment on TDA4 Target

The Robotics SDK runs in a Docker container environment on Processor SDK Linux. In the Robotics SDK Docker environment, ROS and necessary libraries and tools are installed.

Following [this link](https://docs.docker.com/get-started/#test-docker-installation), check that Docker and network work correctly on the Processor Linux on the TDA4 target.

It will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`. The following two sections describe the Docker environment setup, details of building, and running apps under Docker, for ROS 1 and ROS 2, respectively.

#### [Docker Setup for ROS 1 Melodic](./setting_docker_ros1.md)

#### [Docker Setup for ROS 2 Foxy](./setting_docker_ros2.md)

---
**NOTE**

**Proxy Network**: If the board running the Docker container is behind a proxy server, the default settings for downloading files and installing packages via `apt-get` may not work. If you are running the board from TI network, docker build and run scripts will automatically detects and configures necessary proxy settings. For other cases, you may need to modify the configuration files under `docker/proxy` folder and to add the custom proxy settings required for your network, and relevant port of `docker_build_rosX.sh` and `docker_run_rosX.sh`

**Docker Start**: After "`docker build`" is completed, it is important to use `docker_run_rosX.sh` script to start a Docker container, since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the Jacinto device. Please note that `docker_run_rosX.sh` includes "`--rm`" argument. Just remove "`--rm`" argument in `docker_run_rosX.sh` in case you want to do "`docker commit`" after exiting a Docker container. A short information about several useful Docker commands is provided in [this link](http://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-sk-tda4vm/08_00_01_10/exports/docs/docker_environment.html#additional-docker-commands).

**Switching between ROS 1 and ROS 2 Docker Containers**: When the applications are built under the ROS1 container, two directories, {build, devel} are created under **ros_ws** directory under J7 host and simularly {build, install, log} directories are created when applications are built under ROS2 container. Since the containers share the common space **ros_ws** on J7 host, theese directories need to be removed if switching between ROS1 and ROS2 containers.

---
