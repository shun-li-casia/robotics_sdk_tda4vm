
Docker Setup for ROS 1
======================

## 1. Set Up Docker Environment on the TDA4

In ROS 1 Docker container environment, ROS Noetic and necessary libraries and tools are installed.

1. To generate the scripts for building and running a Docker image for ROS 1 Noetic:
    ```
    root@j7-evm:~/j7ros_home$ make scripts ROS_VER=1
    ```
    Please make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

2. To build the Docker image:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_build_ros1.sh
    ```
    This step will take several minutes depending on the network speed, and once "docker build" is completed, you can check the resulting Docker image with `docker images`.

3. To start/run the Docker container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh
    ```
    It is important to use `docker_run_ros1.sh` script to start a Docker container, since the script includes all the necessary settings to leverage all the cores and hardware accelerators of the TDA4 device.

4. To build the ROS applications, inside the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ catkin_make --source /opt/robotics_sdk/ros1
    root@j7-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```

## 2. Set Up Docker Environment on the Remote PC for Visualization

You can choose any folder, but `init_setip.sh` script sets up `${HOME}/j7ros_home` as the working directory.

1. To generate the scripts for building and running a Docker image for ROS 1 Noetic:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1
    ```
    If the Ubuntu PC uses a Nvidia GPU driver, please add one more argument `GPUS=y` as follows:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=1 GPUS=y
    ```
    Please make sure that two bash scripts, `docker_build_ros1.sh` and `docker_run_ros1.sh`, are generated.

2. To build the ROS 1 Docker image:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros1.sh
    ```
    It will take several minutes in building the Docker image. Once "docker run" completes, the resulting Docker image can be listed with "docker images".

3. ROS network setting: We need to set up two environment variables which will be passed to the Docker container and then used in configuring ROS network settings. Please update the following two lines in `setup_env_pc.sh`:
    ```
    export J7_IP_ADDR=<J7_IP_address>
    export PC_IP_ADDR=<PC_IP_address>
    ```
    `<J7_IP_address>` can be found by running `make ip_show` on a TDA4 terminal.

    Then, source the updated script before executing `docker_run_ros1.sh` (in the next step):
    ```
    user@pc:~/j7ros_home$ source ./setup_env_pc.sh
    ```

4. Start/Run the ROS 1 Docker container:
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros1.sh
    ```

5. Build the ROS nodes for visualization:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ catkin_make --source src/robotics_sdk/ros1
    root@pc-docker:~/j7ros_home/ros_ws$ source devel/setup.bash
    ```

## 3. Run Demo Applications

Table below summarizes the launch commands that you can use in the Docker container for each demo, on the TDA4, and on the remote visualization PC. For more details, see the following subsections.

Demo (Input Source) | Launch command on TDA4          | Launch command on Remote Visualization PC
--------------------|---------------------------------|-------------------------------------------
Stereo Vision (ROSBAG)     | roslaunch ti_sde bag_sde.launch  | roslaunch ti_viz_nodes rviz_sde.launch
Stereo Vision (ZED camera) | roslaunch ti_sde zed_sde.launch  | same as above
Stereo Vision with point-cloud (ROSBAG)     | roslaunch ti_sde bag_sde_pcl.launch  | roslaunch ti_viz_nodes rviz_sde_pcl.launch
Stereo Vision with point-cloud (ZED camera) | roslaunch ti_sde zed_sde_pcl.launch  | same as above
Semantic Segmentation CNN (ROSBAG)      | roslaunch ti_vision_cnn bag_semseg_cnn.launch   | roslaunch ti_viz_nodes rviz_semseg_cnn.launch
Semantic Segmentation CNN (ZED camera)  | roslaunch ti_vision_cnn zed_semseg_cnn.launch   | same as above
Semantic Segmentation CNN (Mono camera) | roslaunch ti_vision_cnn gscam_semseg_cnn.launch  | same as above
Object Detection CNN (ROSBAG)      | roslaunch ti_vision_cnn bag_objdet_cnn.launch   | roslaunch ti_viz_nodes rviz_objdet_cnn.launch
Object Detection CNN (ZED camera)  | roslaunch ti_vision_cnn zed_objdet_cnn.launch   | same as above
Object Detection CNN (Mono camera) | roslaunch ti_vision_cnn gscam_objdet_cnn.launch  | same as above
3D Obstacle Detection (ROSBAG)     | roslaunch ti_estop bag_estop.launch  | roslaunch ti_viz_nodes rviz_estop.launch
3D Obstacle Detection (ZED camera) | roslaunch ti_estop zed_estop.launch  | same as above
Visual Localization (ROSBAG)       | roslaunch ti_vl bag_visloc.launch    | roslaunch ti_viz_nodes rviz_visloc.launch

In the following, **[TDA4]** and **[PC]** indicate where the step(s) should be launched: either on the TDA4 target, or on the PC.

### 3.1. Run Stereo Vision Application

1. **[TDA4]** To launch `ti_sde` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde.launch
    ```

### 3.2. Run Stereo Vision Application with Point-Cloud Enabled

1. **[TDA4]** To launch `ti_sde` node with point-cloud enabled on a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_sde bag_sde_pcl.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_sde bag_sde_pcl.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde_pcl.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_sde_pcl.launch
    ```
### 3.3. Run Semantic Segmentation CNN Application

1. **[TDA4]** To launch semantic segmentation demo with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_semseg_cnn.launch
    ```
    To process the image stream from a ZED stereo camera or USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `gscam_semseg_cnn.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC::
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_semseg_cnn.launch
    ```

### 3.4. Run Object Detection CNN Application

1. **[TDA4]** To launch object detection demo with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vision_cnn bag_objdet_cnn.launch
    ```
    To process the image stream from a ZED stereo camera or USB mono camera, replace the launch file with `zed_semseg_cnn.launch` or `gscam_semseg_cnn.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_objdet_cnn.launch
    ```

### 3.5. Run 3D Obstacle Detection Application

1. **[TDA4]** To launch `ti_estop` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_estop bag_estop.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_estop bag_estop.launch
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_estop.launch` in the above.

2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_estop.launch
    ```

### 3.6. Run Visual Localization Application

1. **[J7]** To launch `ti_vl` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the Docker:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ roslaunch ti_vl bag_visloc.launch
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros1.sh roslaunch ti_vl bag_visloc.launch
    ```
2. **[PC]** For visualization, in the ROS 1 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ roslaunch ti_viz_nodes rviz_visloc.launch
    ```

### 3.7. Build and Run Hector SLAM Application

Many open-source SLAM algorithms can run on TDA4. Among them, it is demonstrated how to setup and run Hector SLAM on 2D Lidar data. Please refer to [Hector SLAM Application](../ros1/slam/README.md) for details.
