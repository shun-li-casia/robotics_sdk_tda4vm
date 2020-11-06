How to Set Up TI OpenVX + ROS Environment
=========================================

## Prerequisites

### Hardward: J7/TDA4x Family Processors
https://www.ti.com/processors/automotive-processors/featured-platform.html

### J7 Host Linux: Jacinto Processor SDK Linux Version 7.1.0 or Later
https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-jacinto7/latest/exports/docs/linux/index.html

Currently testing with pre-lease version of PSDKLA 7.1.0

<!-- ================================================================================= -->
## Clone Git Repository
```sh
# Working directory
WORK_DIR=$HOME/j7ros_home
# catkin workspace
CATKIN_WS=$WORK_DIR/catkin_ws

mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src

# git clone
# Not yet active >>> TODO
git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git jacinto_ros_perception
```

## Configure J7 Host Linux & Install TI Vision Apps Library

For convenience, set up following soft-links:
```sh
cd $WORK_DIR
ln -s $CATKIN_WS/src/jacinto_ros_perception/docker
ln -s docker/Makefile
```

### Configure Host Linux
For configuring the host Linux, run the following in `$WORK_DIR`:
```sh
make opkg_config
```

### Install TI Vision Apps Library

To install the TI Vision Apps Library, run the following in `$WORK_DIR`:
```sh
make opkg_repo_download
make ipk_install
```

<!-- ================================================================================= -->
## Build Docker Image

The IP address that is assigned to the J7 host Linux will be required in setting network for the ROS applications.  The IP address is automatically parsed in `Makefile`, to check:
```
make ip_show
```
**Note**: It is highly recommended to use a "static IP" for the J7 EVM to make ROS network setting easy.

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

<!-- ================================================================================= -->
## Run Docker Image
To run the docker image:
```
./docker_run.sh
```

For reference, below shows `docker_run.sh`:
```sh
root@j7-evm:~/j7ros_home# cat docker_run.sh
#!/bin/bash
DOCKER_TAG=j7ros
DOCKER_DIR=/home/root/j7ros_home/catkin_ws/src/jacinto_ros_perception/docker
IP_ADDR=$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)
if [ "$#" -lt 1 ]; then
    COMMAND=/bin/bash
else
    COMMAND="$@"
fi
docker run -it --rm \
    -v /home/root/j7ros_home:/root/j7ros_home \
    -v /usr:/host/usr:ro \
    -v /dev:/dev \
    --privileged \
    --network host \
    --device-cgroup-rule='c 238:* rmw' \
    --env J7_IP_ADDR=$IP_ADDR \
    --env-file $DOCKER_DIR/env_list.txt \
      $DOCKER_TAG $COMMAND
```

Some explanation:

* `-v /home/root/j7ros_home:/root/j7ros_home`: `$WORK_DIR` is volume-mapped to `/root/j7ros_home` in the Docker container, under the folder all the source codes for the project will be available. Therefore, any changes made to `$WORK_DIR` will be accessible both inside the container or on the host Linux

* `--env J7_IP_ADDR=$IP_ADDR`: IP address is passed as an environment variable `J7_IP_ADDR` to the container.

* `--env-file $DOCKER_DIR/env_list.txt`: pass other environment variables that are specified in `$DOCKER_DIR/env_list.txt` for the ROS applications are passed to the container.

To exit from inside the Docker container, type `exit` at the command line.


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
├── opkg-repo_7.1.0-r0.0.tar.gz
├── ros-bag_2020_0914.tar.gz
├── tidl-semseg-model_1.3.0.0.tar.gz
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

### Run Semantic Segmentation CNN App
To launch `ti_semseg_cnn` along with `rosbag play <ROSBAG>`, run the following in `$WORK_DIR` on the J7 host Linux:
```sh
./docker_run.sh roslaunch ti_semseg_cnn bag_semseg_cnn.launch
```

Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
```sh
roslaunch ti_semseg_cnn bag_semseg_cnn.launch
```
