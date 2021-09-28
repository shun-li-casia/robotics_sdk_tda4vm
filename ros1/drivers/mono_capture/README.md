USB Mono Camera ROS Node
========================
USB mono camera (Webcam) ROS node based on OpenCV VideoCapture API for publishing raw image and its camera_info.

**NOTE**: This ROS node currently is tested only with Logitech C920 and C270 webcams in 'YUYV' (YUYV 4:2:2) mode.
'MJPG' (Motion-JPEG) mode is not yet enabled and tested.

## Usage

1. Camera Calibration: A Python tool for camera calibration is provided: `scripts/camera_calibration.py`. This calibration tool takes a set of checkcerboard images captured with the target USB camera, and generates `camera_info.yaml`. Example calibration images are provided. While setting up the Docker environment, the calibration images are downloaded under `$HOME/j7ros_home/data/calib_images`. Below is the usage of the camera calibration tool, and for examples, please see `scripts/calib_C920.sh`.
    ```
    usage: camera_calibration.py [-h] [-p PATH] [-e EXT] [-r PTS_ROWS] [-c PTS_COLUMNS] [-s SQUARE_SIZE] [-v VISUALIZE] [-f SCALING_FACTOR] [-o OUT_FNAME] [-n CAMERA_NAME]

    optional arguments:
    -h, --help            show this help message and exit
    -p PATH, --path PATH  Path to the directory storing images for calibration
    -e EXT, --ext EXT     Extension of the files
    -r PTS_ROWS, --pts_rows PTS_ROWS
                            Number of inner corners per row
    -c PTS_COLUMNS, --pts_columns PTS_COLUMNS
                            Number of inner corners per column
    -s SQUARE_SIZE, --square_size SQUARE_SIZE
                            Size of a square (the edge, by mm) of the checker board
    -v VISUALIZE, --visualize VISUALIZE
                            Visualize the checker board or not
    -f SCALING_FACTOR, --scaling_factor SCALING_FACTOR
                            scaling factor after reading images
    -o OUT_FNAME, --out_fname OUT_FNAME
                            Path to the output file to store result (in YAML format)
    -n CAMERA_NAME, --camera_name CAMERA_NAME
                            Camera name
    ```

2. Rectification Map Generation: A Python tool for generating rectification map is provided: `scripts/generate_rect_map_mono.py`. This tool takes the `camera_info` YAML file, and generates a binary file for undistortion & rectification look-up-table (LUT) which is required in offloading the undistortion/rectification on J7 VPAC/LDC hardware accelerator. Below is the usage of the tool, and for examples, please see `scripts/calib_C920.sh`.

    ```
    usage: generate_rect_map_mono.py [-h] [--input INPUT] [--name NAME]

    optional arguments:
    -h, --help            show this help message and exit
    --input INPUT, -i INPUT
                            YAML file name for camera_info. The file should be stored under ../config folder
    --name NAME, -n NAME  Camera name
    ```

5. Build the USB mono camera ROS node
    ```
    cd $ROS_WS
    # ROS1
    catkin_make
    source devel/setup.bash
    # ROS2
    colcon build
    source install/setup.bash
    ```

5. Launch the mono camera node
    ```
    # ROS1
    roslaunch mono_capture mono_capture.launch
    # ROS2
    ros2 launch mono_capture mono_capture_launch.py
    ```

## Launch File Parameters

 Parameter    | Description                                                               | Value
--------------|---------------------------------------------------------------------------|-------------------------
 model_str    | Camera model string                                                       | string
 device_name  | camera device name. Typically `/dev/video1` on the target with TI Edge AI | string
 camera_mode  | Camera mode                                                               | 'FHD' (1920x1080)
 _            | _                                                                         | 'HD' (1280x720)
 _            | _                                                                         | 'VGA' (640x480)
 frame_rate   | Frame rate at which raw images are published                              | int
 encoding     | image encoding                                                            | 'yuv422' (default) or 'bgr8'
 topic_ns     | topic namespace                                                           | 'camera' (default)

**NOTE**: When `encoding` is set to 'yuv422', the pixel format YUV422::YUYV from the webcam is converted to YUV422::UYVY format considering the compatibility with LDC hardware accelerator.
