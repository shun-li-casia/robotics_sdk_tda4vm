ZED Stereo Camera ROS Node
==========================
This is a stereo camera ROS node for ZED camera, based on OpenCV VideoCapture API. The ROS node publishes left and right raw images and their camera_info.

## Usage

1. Obtain the factory camera calibration data file for your ZED camera with a script provided. Find the serial number of the ZED camera. The serial number can be found on the ZED camera box.
    ```
    cd $SDK_DIR/tools/stereo_camera
    ./download_calib_file.sh <serial_number>
    ```
    where `<serial_number>` is the serial number of the ZED camera.

    Then place the downloaded calibration data file (`SNxxxx.conf`) under `config` folder.

2. Generate `camera_info` YAML files, and undistortion & rectification look-up-table (LUT) files which are required in offloading the undistortion/rectification on TDA4 VPAC/LDC hardware accelerator.

    Run the following script:
    ```
    cd $SDK_DIR/tools/stereo_camera
    python3 generate_rect_map.py -i SNxxxx.conf -m <camera_mode> -p <output_folder_path>
    ```
    where
    * `SNxxxx.conf` is the factory calibration data file obtained from Step 1, and
    * `<camera_mode>` is camera mode. Valid `<camera_mode>`: 2K, FHD, FHD2, HD, HD2, VGA (see ``Launch File Parameters`` section for description). If the `-m` argument is not provided, by default the tool will iterate for the following three camera modes: HD, HD2, FHD and FHD2.

    This script parses the calibration data and generates the following files under `<output_folder_path>`:

    * `<SN_string>_<camera_mode>_camera_info_{left,right}.yaml`: `camera_info` for left and right raw image,
    * `<SN_string>_<camera_mode>_remap_LUT_{left,right}.bin`: undistortion and rectification remap LUT for left and right raw images.

3. Update `launch/zed_capture.launch` (`launch/zed_capture_launch.py` in ROS2. Remember to run `colcon build` after any modification) to modify `zed_sn_str` parameter. Also update relevant parameters in `nodes/*/params.yaml` files for each of demo applications.

4. Build the ZED camera ROS node
    ```
    cd $ROS_WS
    # ROS1
    catkin_make --source /opt/robotics_sdk/ros1
    source devel/setup.bash
    # ROS2
    colcon build --base-paths /opt/robotics_sdk/ros2
    source install/setup.bash
    ```

5. Launch the ZED camera node
    ```
    # ROS1
    roslaunch zed_capture zed_capture.launch
    # ROS2
    ros2 launch zed_capture zed_capture_launch.py
    ```

*** Note ***

For the cases where ZED cameras need to be recalibrated, we provides the OpenCV based calibration tool. For more information, refer to [Stereo Camera Calibration](../../../tools/stereo_camera/calibration/README.md).

## Launch File Parameters

 Parameter     | Description                                                               | Value
---------------|---------------------------------------------------------------------------|-------------------------
 zed_sn_str    | ZED camera serial number string                                           | string
 device_name   | camera device name. Typically `/dev/video2` on the target                 | string
 camera_mode   | ZED camera mode                                                           | '2K' (2208x1242)
 _             | _                                                                         | 'FHD' (1920x1080)
 _             | _                                                                         | 'FHD2' (1920x1024)
 _             | _                                                                         | 'HD' (1280x720)
 _             | _                                                                         | 'HD2' (1280x720)
 _             | _                                                                         | 'VGA' (672x376)
 frame_rate    | frame rate at which raw images are published                              | int
 encoding      | image encoding                                                            | 'yuv422' (default) or 'bgr8'

'FHD2' is a newly added mode that provides 1920x1024 resolution that is supported by the stereo depth engine (SDE) HWA. The images are obtained by cropping top and bottom evenly the original 1080p images captured from the 'FHD' mode.

'HD2' is a newly added mode that provides 720p resolution for experiments that need a longer focal length than the native 'HD'. The images are obtained by center-cropping the original 1080p images captured from the 'FHD' mode.

When `encoding` is set to 'yuv422', the pixel format YUV422::YUYV from the ZED camera is converted to YUV422::UYVY format considering the compatibility with LDC hardware accelerator.

## Data Collection Steps

1. Connect the ZED camera to UBS 3 port (a USB type-A to type-C adaptor is required on TDA4 EVM). Check if the camera is recognized with `ls /dev/video*`. In a Ubuntu PC with a built-in webcam, the stereo camera can be recognized as `/dev/video2`.

2. If necessary, update `launch/zed_capture.launch` to change video device, camera mode, frame rate, and etc.

3. On the first terminal, launch the ZED capture node with following, and keep it running:
    ```
    $ roslaunch zed_capture zed_capture.launch
    ```

4. On the second terminal, to capture into ROS bag files, run one of two examples below

    ```
    # Collect 15 seconds of data and stop itself
    $ roslaunch zed_capture recordbag.launch
    # Save into a series of bag files, each keeping 15 seconds of data, until terminated with Ctrl+C
    $ roslaunch zed_capture recordbag_split.launch
    ```

5. (Optional to check the ROS topics) On 3rd terminal,
    ```
    $ rostopic list
    $ rqt_image_view /camera/left/image_raw
    ```

By default, ROS bag files are stored under `${HOME}/.ros` folder.

**Note**: It is not recommended combining the `zed_capture` node and ROSBAG capture into a single launch file, since it takes some several seconds for the ZED camera to settle down its ISP tuning after the ZED camera node is started.
