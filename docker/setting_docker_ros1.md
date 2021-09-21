
Docker Setup for ROS 1
======================

## 1. Set Up Docker Environment on TDA4 host

In ROS 1 Docker, ROS Melodic and necessary libraries and tools are installed.

1. To generate bash scripts for building and running a Docker image for the Robotics Kit:
    ```
    root@j7-evm:~/j7ros_home$ make scripts ROS_VER=1
    ```
    Make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

2. To build the Docker image:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_build_ros1.sh
    ```

3. To start the Docker container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh
    ```

4. To build the ROS applications, inside the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ catkin_make
    root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```

## 2. Set Up Docker Environment on PC

You can choose any folder, but this section assumes installation under `${HOME}/j7ros_home`.

1. Clone the GIT repository:
    ```
    user@pc:~$ wget https://git.ti.com/cgit/processor-sdk-vision/jacinto_ros_perception/tree/initial_setup.sh
    user@pc:~$ source initial_setup.sh
    ```
    <!-- ```
    user@pc:~$ ROS_WS=$HOME/j7ros_home/ros_ws
    user@pc:~$ mkdir -p $ROS_WS/src
    user@pc:~$ cd $ROS_WS/src
    git clone https://git.ti.com/git/processor-sdk-vision/jacinto_ros_perception.git
    ``` -->

<!-- 2. For convenience, set up soft-links:
    ```
    root@pc:~/j7ros_home$ ln -sf ros_ws/src/jacinto_ros_perception/docker/Makefile
    user@pc:~/j7ros_home/ros_ws$ ln -sf src/jacinto_ros_perception/setup_env_pc.sh
    ``` -->

2. To generate bash scripts for building and running a Docker image for the Robotics Kit:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1
    ```
    If the Ubuntu PC uses a Nvidia GPU driver, please add one more argument `GPUS=y`:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1 GPUS=y
    ```
    Make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

3. To build the Docker image, in `$WORK_DIR` run:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros1.sh
    ```
    It will take several minutes to build the Docker image. The Docker image built can be listed with "docker images".

4. Run the Docker container
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros1.sh
    ```

5. Build the ROS nodes for visualization:
    ```
    user@pc:~/j7ros_home/ros_ws/src$ cd $ROS_WS
    user@pc:~/j7ros_home/ros_ws$ catkin_make
    ```

6. ROS network setting: Update the following lines in `setup_env_pc.sh`:
    ```
    PC_IP_ADDR=<PC_IP_address>
    J7_IP_ADDR=<J7_IP_address>
    ```
    `<J7_IP_address>` can be found by running `make ip_show` on a TDA4 terminal.

    To set up the PC environment, run the following inside the Docker container:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ source setup_env_pc.sh
    ```
    **NOTE**: Make sure running this script before launching vizualization on the PC.

## 3. Run Demo Applications

In the follolwing, **[J7]** and **[PC]** indicate the steps to run on the TDA4/J7 target and on the PC, respectively.
### 3.1. Run Stereo Vision Application

1. **[J7]** To launch `ti_sde` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde.launch` in the above.

2. **[PC]** For visualization, on the PC (Make sure `source setup_env_pc.sh` ahead):
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde.launch
    ```

### 3.2. Run Stereo Vision Application with Point-Cloud Enabled

1. **[J7]** To launch `ti_sde` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde_pcl.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde_pcl.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde_pcl.launch` in the above.

2. **[PC]** For visualization, on the PC (Make sure `source setup_env_pc.sh` ahead):
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde_pcl.launch
    ```
### 3.3. Run Semantic Segmentation CNN Application

1. **[J7]** To launch semantic segmentation demo with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    To process the image stream from a ZED stereo camera or USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `mono_semseg_cnn.launch` in the above.

2. **[PC]** For visualization, on the PC:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_semseg_cnn.launch
    ```

### 3.4. Run Objdect Detection CNN Application

1. **[J7]** To launch object detection demo with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    To process the image stream from a ZED stereo camera or USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `mono_semseg_cnn.launch` in the above.

2. **[PC]** For visualization, on the PC:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_objdet_cnn.launch
    ```

### 3.5. Run 3D Obstacle Detection Application

1. **[J7]** To launch `ti_estop` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_estop bag_estop.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_estop bag_estop.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_estop.launch` in the above.

2. **[PC]** For visualization, on the PC:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_estop.launch
    ```

### 3.6. Run Visual Localization Application

1. **[J7]** To launch `ti_vl` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vl bag_visloc.launch
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vl bag_visloc.launch
    ```

2. **[PC]** For visualization, on the PC:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_visloc.launch
    ```

### 3.7. Build and Run Hector SLAM Application

Many open source SLAM algorithms can run on J7. Among them, it is demonstrated how to setup and run Hector SLAM on 2D Lidar data. Please refer to [Hector SLAM Application](./ros1/slam/README.md) for details.
