How to Set Up TI OpenVX + ROS Environment
=========================================

## Prerequisites & Dependency

### Hardward: J7/TDA4x Family Processors
https://www.ti.com/processors/automotive-processors/featured-platform.html

### J7 Processor SDK RTOS Version 7.1.0
Currently testing with Prebuilt Package of J7 Processor SDK RTOS 7.1.0 RC9 [LINK](http://gtweb.dal.design.ti.com/nightly_builds/PSDKRA_INSTALLER/57-2020-11-19_11-41-23/artifacts/output/webgen/publish/PROCESSOR-SDK-RTOS-J721E/07_01_00_09/)

Download [Pre-built Package](http://gtweb.dal.design.ti.com/nightly_builds/PSDKRA_INSTALLER/57-2020-11-19_11-41-23/artifacts/output/webgen/publish/PROCESSOR-SDK-RTOS-J721E/07_01_00_09/exports/ti-processor-sdk-rtos-j721e-evm-07_01_00_09-prebuilt.tar.gz) and instrall to a SD card by referring to the instruction on this page [LINK](http://gtweb.dal.design.ti.com/nightly_builds/PSDKRA_INSTALLER/57-2020-11-19_11-41-23/artifacts/output/webgen/publish/PROCESSOR-SDK-RTOS-J721E/07_01_00_09/exports/docs/psdk_rtos/docs/user_guide/out_of_box_j721e.html)

### Terminal Session to J7

Once finding the IP address assigned to J7 EVM (e.g., using a serial port communications program, for example, `minicom`), connect to J7 Linux with SSH:

```
ssh root@<J7_IP_address>
```

**Note**: It is highly recommended to use a "static IP" for the J7 EVM to make ROS network setting easy.


<!-- ================================================================================= -->
## Clone Git Repository

```sh
# project home directory
WORK_DIR=$HOME/j7ros_home
# catkin workspace
CATKIN_WS=$WORK_DIR/catkin_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src

# clone the project GIT repository
# Below is accessible only in TI network:
git clone ssh://git@bitbucket.itg.ti.com/processor-sdk-vision/jacinto_ros_perception.git
# To be be available on the following GIT repository:
# git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git
```

## Configure J7 Host Linux & Install TI Vision Apps Library

For convenience, set up following soft-links:
```sh
cd $WORK_DIR
ln -s $CATKIN_WS/src/jacinto_ros_perception/docker/Makefile
ln -s docker/Makefile
```

<!-- ================================================================================= -->
## Build Docker Image

The IP address that is assigned to the J7 host Linux will be required in setting network for the ROS applications. The IP address is automatically parsed in `Makefile`. To check the IP address:
```
make ip_show
```

### Generate Scripts
To generate bash scripts for building and running a Docker image for the project:
```
make scripts
```
Make sure that two bash scripts named `docker_build.sh` and `docker_run.sh` are generated.

### Docker & Network Testing
Following https://docs.docker.com/get-started/#test-docker-installation,
try the following to check whether Docker and network work correctly on the J7 host Linux:
```
docker run hello-world
```
Make sure that following message shows up in the terminal:
```
Hello from Docker!
This message shows that your installation appears to be working correctly.
```

### Build Docker Image
To build the Docker image, at `$WORK_DIR` run:
```
./docker_build.sh
```

It will take several minutes to build the Docker image. To check the Docker images built:
```
docker images
```

It will show docker images information, for example, as shown below:
```
root@j7-evm:~/j7ros_home# docker images
REPOSITORY          TAG                        IMAGE ID            CREATED             SIZE
j7ros               latest                     34ef46e34368        2 minutes ago       1.6GB
arm64v8/ubuntu      18.04                      0ccb47f043f5        3 months ago        57.8MB
```

**Note**: The Docker image that is built using `Makefile` provided in the GIT repository will include minimal number of ROS packages on which the ROS package(s) under `$CATKIN_WS/src` have dependency. In case when more ROS package(s) are added under the folder, it is required to re-build a Docker image using the `Makefile`.

<!-- ================================================================================= -->
## Run Docker Image
To run the docker image:
```
./docker_run.sh
```

To exit from inside the Docker container, type `exit` at the command line.

<!-- ================================================================================= -->
## Setting Up Remote PC for Visualization

Open another terminal on Ubuntu PC to set up environments for visualization.

### Clone GIT repository
```sh
# catkin workspace
CATKIN_WS=$HOME/j7ros_home/catkin_ws

mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src

# git clone
# Below is accessible only in TI network:
git clone ssh://git@bitbucket.itg.ti.com/processor-sdk-vision/jacinto_ros_perception.git
# To be be available on the following GIT repository:
# git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git
```

### Build ROS nodes
```
cd $CATKIN_WS
catkin_make
```

### PC Environment Setting
For convenience, set up following soft-links:
```sh
cd $CATKIN_WS
ln -s src/jacinto_ros_perception/setup_env_pc.sh
```

Update the following lines in `setup_env_pc.sh` for setting the ROS network:
```
PC_IP_ADDR=<PC_IP_address>
J7_IP_ADRR=<J7_IP_address>
```

`<J7_IP_address>` can be found by running the following on the J7 terminal:
```
make ip_show
```

To set up the PC environment, run the following:
```
source setup_env_pc.sh
```

### Checking ROS topics
After launching ROS nodes on the J7, when the network is set correctly, we can check the all the topics published with:
```
rostopic list
```

<!-- ================================================================================= -->
## ROS Applications

### Download Data
To download data files, run the following in `$WORK_DIR` on the J7 host Linux:
```sh
make data_download
```
Several tarballs are downloaded and uncompressed under `$WORK_DIR/data`, with folder structure, for example, as shown below:

```
$WORK_DIR/data
├── ros_bag
│   └── 2020-09-14-12-31-43.bag
└── tidl_semseg_model
    ├── city_deeplabv3lite_mobilenetv2_tv_1.bin
    └── city_deeplabv3lite_mobilenetv2_tv.bin
```

### Build ROS Applications

Inside the Docker container:
```sh
cd $CATKIN_WS
catkin_make
source devel/setup.bash
```

### Run Stereo Vision App
To launch `ti_sde` along with `rosbag play <ROSBAG>`, run the following in `$WORK_DIR` on the J7 host Linux:
```sh
./docker_run.sh roslaunch ti_sde bag_sde.launch
```

Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
```sh
roslaunch ti_sde bag_sde.launch
```

For visualization, on the PC (see the above for setting PC environment):
```
roslaunch ti_sde rviz.launch
```

### Run Semantic Segmentation CNN App
To launch `ti_semseg_cnn` along with `rosbag play <ROSBAG>`, run the following in `$WORK_DIR` on the J7 host Linux:
```sh
./docker_run.sh roslaunch ti_semseg_cnn bag_semseg_cnn.launch
```

Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
```sh
roslaunch ti_semseg_cnn bag_semseg_cnn.launch
```

For visualization, on the PC (see the above for setting PC environment):
```
roslaunch ti_semseg_cnn rviz.launch
```

