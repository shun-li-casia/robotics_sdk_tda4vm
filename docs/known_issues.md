Limitations and Known Issues
============================

1. RViz visualization is displayed on a remote Ubuntu PC.
2. Ctrl+C termination of a ROS node or a ROS launch can be sometimes slow and escalated to SIGTERM. This behavior happens more often in the ROS 2 container.
3. Stereo Vision Demo
    * Output disparity map may have artifacts that are common to block-based stereo algorithms. e.g., noise in the sky, texture-less area, repeated patterns, etc.
    * While the confidence map from SDE has 8 values between 0 (least confident) to 7 (most confident), the confidence map from the multi-layer SDE refinement has only 2 values, 0 and 7. Therefore, it would not appear as fine as the SDE's confidence map.
4. The default semantic segmentation model used in `ti_vision_cnn` and `ti_estop` nodes was trained with Cityscapes dataset first, and re-trained with a small dataset collected from a particular stereo camera (ZED camera, in HD mode) for a limited scenarios with coarse annotation. Therefore, the model can show limited accuracy performance if a different camera model is used and/or when it is applied to different environment scenes.
5. Launching a demo in ROS 2 environment with "ros2 bag play" in a single launch script currently not stable. It is recommended to launch “ros2 bag play” in a separate terminal. The demos in ROS 2 container currently run more stable with live cameras (ZED stereo camera or USB mono camera).
6. The USB mono camera capture node ('mono_capture') currently is tested with Logitech C920 and C270 webcams in 'YUYV' (YUYV 4:2:2) mode. To use the USB camera in 'MJPG' (Motion-JPEG) mode, it is recommended to use the GStreamer based camera capture node ('gscam' or 'gscam2') instead.
7. ZED camera is sometimes recognized as USB 2.1 device. The ZED camera should be connected to USB 3.0 for the demos, which can be checked with following command:
    ```text
    root@tda4vm-sk:~# lsusb -v -d 2b03:f582 | grep bcdUSB
      bcdUSB               3.00
    ```
