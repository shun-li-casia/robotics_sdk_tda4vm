Docker Setup for ROS 2
======================

In the ROS 2 Docker images both for TDA4 and Ubuntu PC, following two popular DDS implementations are installed:

* eProsima Fast DDS (default DDS for ROS 2 Foxy)
* Eclipse Cyclone DDS (additionally installed)

To make the out-of-box demos run smooth, the following DDS selection are made in the ROS 2 Docker containers by default on each platform.

Platform | DDS Choice        | Note
---------|-------------------|---------------------------------------------------------------------
TDA4 target      | Eclipse Cyclone DDS | Provides better performance, especially with "rosbag play"
Visualization PC | eProsima Fast DDS   | Provides better `Ctrl+C` response

By setting an environment variable `RMW_IMPLEMENTATION` you can switch the DDS implementation in each launch session. For example,

To use Fast DDS,
```
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 ...
```

To use Cyclone DDS,
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 ...
```

## 1. Set Up Docker Environment on J7 host

In ROS 2 Docker environment, ROS Foxy and necessary libraries and tools are installed.

1. To generate bash scripts for building and running a Docker image for the Robotics Kit:
    ```
    root@j7-evm:~/j7ros_home$ make scripts ROS_VER=2
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

2. To build the Docker image:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_build_ros2.sh
    ```
    This step will take several minutes, and once "docker build" is completed, you can check the resulting Docker image with `docker images`.

3. To start the Docker container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh
    ```

4. To build the ROS applications, inside the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ colcon build --base-paths src/jacinto_ros_perception/ros2
    root@j7-docker:~/j7ros_home/ros_ws$ source install/setup.bash
    ```
    **NOTE**: In case when "colcon build" fails, you can try again after adding `--executor sequential` to the `colcon build` command above.

5. The ROSBAG data downloaded is in ROS 1 specific format and the format and storage mechanism has changed in ROS 2. For converting the ROS 1 bag data to ROS 2 format, use `rosbags-convert <ros1_bag_name.bag>`. This will create a directory <ros_bag_name> and contains the data under this directory. To convert the downloaded ROSBAG file, run the following inside the ROS 2 Docker container.
    ```
    root@j7-docker:~/j7ros_home$ rosbags-convert $WORK_DIR/data/ros_bag/zed1_2020-11-09-18-01-08.bag
    ```

## 2. Set Up Docker Environment on PC for Visualization

You can choose any folder, but this section assumes installation is made under `${HOME}/j7ros_home`.

1. To generate bash scripts for building and running a Docker image for the ROS 2 Docker container:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2
    ```
    If the Ubuntu PC uses a Nvidia GPU driver, please add one more argument `GPUS=y`:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2 GPUS=y
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

2. To build the ROS 2 Docker image:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros2.sh
    ```
    It will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`.

3. Run the ROS 2 Docker container:
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros2.sh
    ```

4. Build the ROS nodes for visualization:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ colcon build
    ```

## 3. Run Demo Applications

Table below summarizes the launch commands that you can use in the Docker container for each demo, on the TDA4/J7, and on the remote visualization PC. For more details, see the following subsections.

<!-- Demo (Input Source) | Launch command on TDA4          | Launch command on Remote Visualization PC
--------------------|---------------------------------|-------------------------------------------
Stereo Vision (ROSBAG)     | ros2 launch ti_sde bag_sde_launch.py  | ros2 launch ti_viz_nodes rviz_sde_launch.py
Stereo Vision (ZED camera) | ros2 launch ti_sde zed_sde_launch.py  | same as above
Stereo Vision with point-cloud (ROSBAG)     | ros2 launch ti_sde bag_sde_pcl_launch.py  | ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py
Stereo Vision with point-cloud (ZED camera) | ros2 launch ti_sde zed_sde_pcl_launch.py  | same as above
Semantic Segmentation CNN (ROSBAG)      | ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
Semantic Segmentation CNN (ZED camera)  | ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py   | same as above
Semantic Segmentation CNN (Mono camera) | ros2 launch ti_vision_cnn mono_semseg_cnn_launch.py  | same as above
Object Detection CNN (ROSBAG)      | ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
Object Detection CNN (ZED camera)  | ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py   | same as above
Object Detection CNN (Mono camera) | ros2 launch ti_vision_cnn mono_objdet_cnn_launch.py  | same as above
3D Obstacle Detection (ROSBAG)     | ros2 launch ti_estop bag_estop_launch.py  | ros2 launch ti_viz_nodes rviz_estop_launch.py
3D Obstacle Detection (ZED camera) | ros2 launch ti_estop zed_estop_launch.py  | same as above
Visual Localization (ROSBAG)       | ros2 launch ti_vl bag_visloc_launch.py    | ros2 launch ti_viz_nodes rviz_visloc_launch.py -->

Demo (Input Source) | Launch command on TDA4          | Launch command on Remote Visualization PC
--------------------|---------------------------------|-------------------------------------------
Stereo Vision (ZED camera) | ros2 launch ti_sde zed_sde_launch.py  | ros2 launch ti_viz_nodes rviz_sde_launch.py
Stereo Vision with point-cloud (ZED camera) | ros2 launch ti_sde zed_sde_pcl_launch.py  | ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py
Semantic Segmentation CNN (ZED camera)  | ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
Semantic Segmentation CNN (Mono camera) | ros2 launch ti_vision_cnn mono_semseg_cnn_launch.py  | same as above
Object Detection CNN (ZED camera)  | ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
Object Detection CNN (Mono camera) | ros2 launch ti_vision_cnn mono_objdet_cnn_launch.py  | same as above
3D Obstacle Detection (ZED camera) | ros2 launch ti_estop zed_estop_launch.py  | ros2 launch ti_viz_nodes rviz_estop_launch.py

**Running Demos with ROSBAG**

It is recommended to launch demos in two terminals on the TDA4 target, and launch `ros2 bag play` in a separate terminal as shown is the following table.

Demo                | ROSBAG launch command on TDA4 (Terminal 1) | Demo launch command on TDA4 (Terminal 2) | Launch command on Remote Visualization PC
--------------------|--------------------------------------------|------------------------------------------|------------------------------------------
Stereo Vision       | ros2 launch ti_sde rosbag_remap_launch.py | ros2 launch ti_sde sde_launch.py | ros2 launch ti_viz_nodes rviz_sde_launch.py
Stereo Vision with point-cloud | same as above  | ros2 launch ti_sde sde_pcl_launch.py             | ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py
Semantic Segmentation CNN      | same as above  | ros2 launch ti_vision_cnn semseg_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
Object Detection CNN           | same as above  | ros2 launch ti_vision_cnn objdet_cnn_launch.py   | ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
3D Obstacle Detection          | same as above  | ros2 launch ti_estop estop_launch.py             | ros2 launch ti_viz_nodes rviz_estop_launch.py

You can use TMUX inside the ROS Docker container to split the current terminal window into multiple panes. Below are some of useful basic keys in using TMUX. Full list of keys can be found, e.g., in [this link](https://tmuxcheatsheet.com/).
* `tmux`: start a tmux session
* `Ctrl + b`, followed by `"`: split pane vertically
* `Ctrl + b`, followed by `↑` or `↓`: switch to pane to the direction
* `Ctrl + b`, followed by `x`: close the current pane

In the following, **[TDA4]** and **[PC]** indicate where the step(s) should be launched: either on the TDA4 target, or on the PC.

### 3.1. Run Stereo Vision Application

1. **[TDA4]** To launch `ti_sde` node with a ZED stereo camera, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_sde zed_sde_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_sde zed_sde_launch.py
    ```

2. **[PC]** For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_sde_launch.py
    ```

### 3.2. Run Stereo Vision Application with Point-Cloud Enabled

1. **[TDA4]** To launch `ti_sde` node with point-cloud enabled on a ZED camera, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_sde zed_sde_pcl_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_sde zed_sde_pcl_launch.py
    ```

2. **[PC]** For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py
    ```

### 3.3. Run Semantic Segmentation CNN Application

1. **[TDA4]** To launch semantic segmentation demo with a ZED stereo camera, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vision_cnn zed_semseg_cnn_launch.py
    ```
    To process the image stream from a USB mono camera, replace the launch file with `mono_semseg_cnn_launch.py` in the above.

2. **[PC]**  For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
    ```

### 3.4. Run Object Detection CNN Application

1. **[TDA4]** To launch object detection demo with a ZED stereo camera, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py
    ```
    To process the image stream from a USB mono camera, replace the launch file with `mono_objdet_cnn_launch.py` in the above.

2. **[PC]**  For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
    ```

### 3.5. Run 3D Obstacle Detection Application

1. **[TDA4]** To launch `ti_estop` node with a ZED stereo camera, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_estop zed_estop_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_estop zed_estop_launch.py
    ```

2. **[PC]**  For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_estop_launch.py
    ```

<!-- ### 3.6. Run Visual Localization Application

1. **[TDA4]** To launch `ti_vl` node with playing back a ROSBAG file, run the following `roslaunch` command **inside** the ROS 2 container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vl bag_visloc_launch.py
    ```
    Alternatively, you can run the following directly on the TDA4 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vl bag_visloc_launch.py
    ```

2. **[PC]**  For visualization, in the ROS 2 container on the PC:
    ```
    root@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_visloc_launch.py
    ``` -->
