Vision CNN: Object Detection
============================

![](docs/objdet_rviz.png)
<br />

This `ti_vision_cnn` node is versitile deep-learning (DL) inference ROS node that is optimized on DL cores and hardware acceelrator of TDA4. The `ti_vision_cnn` node supports compute-intensive DL inference operations including 2D object detection and semantic segmentation. Figure 1 shows the high-level block diagram of the applications around the `ti_vision_cnn` node, which consists of multiple processing blocks that are deployed on hardware accelerators and DSP processors for pre-processing and post-processing in an optimized manner.

For details of block diagram and parameters of `ti_vision_cnn`, please refer to [README.md](./README.md).

## Objdect Detection Demo

### How to Run the Application in ROS 1

**[J7]** To launch object detection demo with playing back a ROSBAG file, run the following inside the Docker container on J7 target:
```
roslaunch ti_vision_cnn bag_objdet_cnn.launch
```
To process the image stream from a ZED stereo camera:
```
roslaunch ti_vision_cnn zed_objdet_cnn.launch
```
To process the image stream from a USB mono camera:
```
roslaunch ti_vision_cnn mono_objdet_cnn.launch
```

**[Visualization on Ubuntu PC]** For setting up environment of the remote PC, please follow "Set Up the Ubuntu PC for Visualization" section of [docker/README.md](../../docker/README.md)

To launch visuzalization:
```
roslaunch ti_viz_nodes rviz_objdet_cnn.launch
```
### How to Run the Application in ROS 2

**[J7]** To launch object detection demo with playing back a ROSBAG file, run the following inside the Docker container on J7 target:
```
ros2 launch ti_vision_cnn bag_objdet_cnn_launch.py
```
To process the image stream from a ZED stereo camera:
```
ros2 launch ti_vision_cnn zed_objdet_cnn_launch.py
```
To process the image stream from a USB mono camera:
```
roslaunch ti_vision_cnn mono_objdet_cnn.launch
```

**[Visualization on Ubuntu PC]** For setting up environment of the remote PC, please follow "Set Up the Ubuntu PC for Visualization" section of [docker/README.md](../../docker/README.md)

```
ros2 launch ti_viz_nodes rviz_objdet_cnn_launch.py
```
### Known Issue

1. The default 2D object detection model (ONR-OD-8080-yolov3-lite-regNetX-1.6gf-bgr-coco-512x512) has initial lodading time of about 20 seconds.