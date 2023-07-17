# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on the target SK board. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

[To be Added].

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.


## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

[To be Added].
