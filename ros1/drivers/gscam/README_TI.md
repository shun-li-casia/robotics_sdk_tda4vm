GStreamer Camera Node for ROS 1
===============================
This GStreamer based camera ROS node is from [https://github.com/ros-drivers/gscam](https://github.com/ros-drivers/gscam). Following changes are made to customize for the use cases in the Robotics SDK:

* Added GStreamer pipelines that use [the GStreamer plugins optimized for TI Processors](https://github.com/TexasInstruments/edgeai-gst-plugins), and added NV12 encoding mode for the output.
* Added an example `camera_info.yaml` for Logitech webcam C920 and associated LDC look-up-table file (required to use the LDC hardware accelerator).
* Added launch files (under 'launch' folder) for Logitech webcam (in MJPG mode and YUYV mode) and OV5640 (in YUYV mode) CSI camera with GStreamer pipelines using the plugins optimized on TI devices.
* Added launch files for IMX390 FPD-Link cameras with GStreamer pipelines using the plugins optimized on TI devices.
* Dependency: following modules are already built and installed in the the Robotics SDK ROS Docker images.
    * [edgeai-tiovx-modules](https://github.com/TexasInstruments/edgeai-tiovx-modules)
    * [edgeai-gst-plugins](https://github.com/TexasInstruments/edgeai-gst-plugins)

```{note}
The customization are made only for the Robotics SDK. It was not verified that all examples from the original GIT repository still work with these changes.
```

## Usage: USB Camera

### Running gscam on the target

1. Camera Calibration and Rectification Map Generation: See corresponding parts of [mono_capture/README.md](../mono_capture/README.md).

2. Build the ROS node:
    ```
    cd $ROS_WS
    catkin_make --source /opt/robotics_sdk/ros1
    source devel/setup.bash
    ```

3. Launch the "gscam" ROS node:
Before launching the "gscam", please make sure to update `device` in the launch file to point to correct camera device (`/dev/videoX`).
    For capturing in MJPG (motion JPEG) mode,
    ```
    roslaunch gscam v4l_mjpg.launch
    ```
    For capturing in YUYV mode,
    ```
    roslaunch gscam v4l_yuv.launch
    roslaunch gscam v4l_ov5640.launch # for OV5640 CSI camera
    ```

```{note}
The measured framerate for the output topic can be less than the framerate set in the launch file, depending on the light condition of the scenes. This is a normal behavior inherited from `v4l2src` and the default ISP settings of the USB webcam.
```

### Visualization on remote PC

By default, the output topic `raw_image` is published in NV12 color format to make more efficient when the node is integrated with the vision vision CNN processing chain (including `ti_vision_cnn` ROS node) on the target. We provide a launch file for visualization on the remote Ubuntu (included in `ti_viz_nodes` ROS package).

In the PC Docker container,
```
roslaunch ti_viz_nodes gscam_nv12.launch
```

## Usage: IMX219 Camera

```{only} tag_j7x
Please follow [the hardware setup section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/latest/exports/docs/devices/TDA4VM/linux/getting_started.html#rpiv2-imx219-raw-sensor).
```
```{only} tag_am62a
Please follow [the hardware setup section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/AM62AX/latest/exports/docs/devices/AM62AX/linux/getting_started.html#rpiv2-imx219-raw-sensor).
```

You can check the device ID and subdev ID for the IMX219 CSI camera attached to the hardware setup by running `/opt/edgeai-gst-apps/scripts/setup_cameras.sh` on the target host Linux. Accordingly please update the launch files below, or it's also possible to pass as launch arguments.

### Running gscam with resizing
To publish the captured images from the IMX219 camera,
```
roslaunch gscam v4l_imx219.launch device_id:=X subdev_id:=Y
```

```{note}
The GStreamer pipeline in `v4l_imx219.launch` includes the ISP plugin (tiovxisp) followed by the image scaler plugin (tiovxmultiscaler) to resize the images to the output resolution. The output images are published in NV12 format.
```

### Visualization on remote PC

For visualization, run the following in the PC Docker container,
```
roslaunch ti_viz_nodes gscam_nv12.launch width:=1280 height:=720
```

## Usage: IMX390 Camera

```{only} tag_j7x
Please follow [the hardware setup section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/TDA4VM/latest/exports/docs/devices/TDA4VM/linux/getting_started.html#imx390-raw-sensor).
```
```{only} tag_am62a
Please follow [the hardware setup section of Edge AI documentation](https://software-dl.ti.com/jacinto7/esd/processor-sdk-linux-edgeai/AM62AX/latest/exports/docs/devices/AM62AX/linux/getting_started.html#imx390-raw-sensor).
```

### Running gscam for raw image capture

You can check the device ID and subdev ID for the IMX390 camera attached to the hardware setup by running `/opt/edgeai-gst-apps/scripts/setup_cameras.sh` on the target host Linux. Accordingly please update the launch files below, or it's also possible to pass as launch arguments.

To publish raw images in the native resolution (1936 x 1096),
```
roslaunch gscam v4l_imx390_raw.launch
```
### Running gscam with rectification and resizing

We also provide a launch file that includes rectification and resizing in the GStreamer pipeline.

````{note}
`v4l_imx390.launch` has `tiovxldc` in the GStreamer pipeline and
`tiovxldc` requires `lut-file`. The LUT file is specific to the camera.
As an example, we provide camera calibration data for a fisheye IMX390 camera
is provided in the form of camera_info YAML file. To generate the LUT files,
please run the following in the Robotics SDK Docker container on the target:
```
$ bash /opt/robotics_sdk/tools/mono_camera/imx390_ldc.sh
```
````

```
# 1080p
roslaunch gscam v4l_imx390.launch width:=1920 height:=1080
# 720p
roslaunch gscam v4l_imx390.launch width:=1280 height:=720
```

```{note}
The GStreamer pipeline in `v4l_imx390.launch` also includes the LDC plugin (tiovxldc). Raw resolution is 1936 x 1096. The LDC plugin performs rectification and then cropping to produce 1920 x 1080 images in NV12 format, followed by MSC plugin (tiovxmultiscaler) to resize the images to the output resolution.
```

### Visualization on remote PC

For visualization, run the following in the PC Docker container,
```
# native resolution (1936 x 1096)
roslaunch ti_viz_nodes gscam_nv12.launch width:=1936 height:=1096
# 1080p
roslaunch ti_viz_nodes gscam_nv12.launch width:=1920 height:=1080
# 720p
roslaunch ti_viz_nodes gscam_nv12.launch width:=1280 height:=720
```
