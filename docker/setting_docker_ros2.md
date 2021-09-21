Docker Setup for ROS 2
======================

## 1. Set Up Docker Environment on J7 host

In ROS 2 Docker environment, ROS Foxy and necessary libraries and tools are installed.

1. To generate bash scripts for building and running a Docker image for the Robotics Kit:
    ```
    root@j7-evm:~/j7ros_home$ make scripts ROS_VER=2
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

2. To build the Docker image, in `$WORK_DIR` run:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_build_ros2.sh
    ```

3. To start the Docker container:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh
    ```

4. To build the ROS applications, inside the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ colcon build
    root@j7-docker:~/j7ros_home/ros_ws$ source install/setup.bash
    ```

5. The ROSBAG data downloaded is in ROS 1 specific format and the format and storage mechanism has changed in ROS 2. For converting the ROS 1 bag data to ROS 2 format, use `rosbags-convert <ros1_bag_name.bag>`. This will create a directory <ros_bag_name> and contains the data under this directory. To convert the downloaded ROSBAG file, run the following insize the ROS 2 Docker container.
    ```
    root@j7-docker:~/j7ros_home$ rosbags-convert $WORK_DIR/data/ros_bag/zed1_2020-11-09-18-01-08.bag
    ```

## 2. Set Up Docker Environment on PC

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

2. To generate bash scripts for building and running a Docker image for the Robotics Kit:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2
    ```
    If the Ubuntu PC uses a Nvidia GPU driver, please add one more argument `GPUS=y`:
    ```
    user@pc:~/j7ros_home$ make scripts ROS_VER=2 GPUS=y
    ```
    Make sure that two bash scripts, `docker_build_ros2.sh` and `docker_run_ros2.sh`, are generated.

3. To build the Docker image, in `$WORK_DIR` run:
    ```
    user@pc:~/j7ros_home$ ./docker_build_ros2.sh
    ```
    It will take several minutes to build the Docker image. The Docker image built can be listed with `docker images`.

4. Run the Docker container
    ```
    user@pc:~/j7ros_home$ ./docker_run_ros2.sh
    ```

5. Build the ROS nodes for visualization:
    ```
    user@pc:~/j7ros_home/ros_ws/src$ cd $ROS_WS
    user@pc:~/j7ros_home/ros_ws$ colcon build
    ```

## 3. Run Demo Applications

### 3.1. Run Stereo Vision Application

1. **[J7]** To launch `ti_sde` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_sde bag_sde_launch.py
    ```
    Alternatively, you can run the following `ros2 launch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_sde bag_sde_launch.py
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde_launch.py` in the above.

2. **[PC]** For visualization, on the Docker container under PC, run the following in `$WORK_DIR`:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_sde_launch.py
    ```

### 3.2. Run Stereo Vision Application with Point-Cloud Enabled

1. **[J7]** To launch `ti_sde` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_sde bag_sde_pcl_launch.py
    ```
    Alternatively, you can run the following `ros2 launch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_sde bag_sde_pcl_launch.py
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_sde_launch.py`  in the above.

2. **[PC]** For visualization, on the Docker container under PC, run the following in `$WORK_DIR`:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_sde_pcl_launch.py
    ```

### 3.3. Run Semantic Segmentation CNN Application

1. **[J7]** To launch semantic segmentation demo with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py
    ```
    Alternatively, you can run the following `ros2 launch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vision_cnn bag_semseg_cnn_launch.py
    ```
    To process the image stream from a ZED stereo camera or a USB mono camera, replace the launch file with `zed_semseg_cnn_launch.py` or `mono_semseg_cnn_launch.py` in the above.

2. **[PC]** For visualization, on the Docker container under PC, run the following in `$WORK_DIR`:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_semseg_cnn_launch.py
    ```

### 3.4. Run Object Detection CNN Application

1. **[J7]** To launch objdect detection demo with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py
    ```
    Alternatively, you can run the following `ros2 launch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py
    ```
    To process the image stream from a ZED stereo camera or a USB mono camera, replace the launch file with `zed_objdet_cnn_launch.py` or `mono_objdet_cnn_launch.py` in the above.

2. **[PC]** For visualization, on the Docker container under PC, run the following in `$WORK_DIR`:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
    ```

### 3.5. Run 3D Obstacle Detection Application

1. **[J7]** To launch `ti_estop` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_estop bag_estop_launch.py
    ```
    Alternatively, you can run the following `ros2 launch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_estop bag_estop_launch.py
    ```
    To process the image stream from a ZED stereo camera, replace the launch file with `zed_estop_launch.py` in the above.

2. **[PC]** For visualization, on the Docker container under PC, run the following in `$WORK_DIR`:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_estop_launch.py
    ```

### 3.6. Run Visual Localization Application

1. **[J7]** To launch `ti_vl` node with playing back a ROSBAG file, run the following in `$WORK_DIR` on the J7 host Linux:
    ```
    root@j7-evm:~/j7ros_home$ ./docker_run_ros2.sh ros2 launch ti_vl bag_visloc_launch.py
    ```
    Alternatively, you can run the following `roslaunch` command **inside** the Docker container:
    ```
    root@j7-docker:~/j7ros_home/ros_ws$ ros2 launch ti_vl bag_visloc_launch.py
    ```

2. **[PC]** For visualization, on the PC:
    ```
    user@pc-docker:~/j7ros_home/ros_ws$ ros2 launch ti_viz_nodes rviz_visloc_launch.py
    ```
