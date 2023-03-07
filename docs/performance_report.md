Performance Report
==================

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1322| NA| NA| 18.75| 687| 530| 1217| 0.0| 0.0| 0.0| 2.0| 2.0| 0| 0| 0| 0| 4.42| 16.49| 0
ti_sde (w/ pcl)| 15.12| 66.1181| NA| NA| 33.8| 1057| 928| 1985| 0.0| 16.0| 0.0| 5.0| 3.0| 6.59| 0| 0| 0| 4.48| 21.30| 0
ti_vision_cnn (semseg)| 15.12| 66.1341| 3.0101| 8.0707| 12.50| 533| 310| 843| 11.0| 1.0| 1.0| 2.0| 1.0| 3.24| 0| 0| 0| 2.15| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.1137| 2.9975| 5.0907| 11.0| 489| 279| 768| 8.0| 0.0| 0.0| 2.0| 1.0| 3.22| 0| 0| 0| 2.13| 0| 0
ti_estop| 15.14| 66.0334| 3.0051| 8.0716| 25.74| 928| 600| 1528| 11.0| 0.0| 0.0| 3.0| 1.0| 3.32| 0| 0| 0| 4.55| 16.31| 0
ti_objdet_range| 13.74| 72.7799| 3.0063| 5.5250| 44.5| 1475| 1092| 2567| 8.0| 0.0| 1.0| 7.0| 3.0| 10.62| 0| 0| 0| 7.26| 25.73| 0
ti_vl| 15.17| 65.9390| 3.0221| 16.7459| 30.22| 1144| 986| 2130| 17.0| 69.0| 2.0| 2.0| 1.0| 1.26| 0| 0| 0| 0.80| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1509| NA| NA| 26.17| 928| 864| 1792| 0.0| 0.0| 0.0| 3.0| 2.0| 0| 0| 0| 0| 4.53| 16.36| 0
ti_sde (w/ pcl)| 15.08| 66.3031| NA| NA| 34.51| 1228| 1123| 2351| 0.0| 16.0| 0.0| 5.0| 3.0| 6.64| 0| 0| 0| 4.52| 20.78| 0
ti_vision_cnn (semseg)| 15.14| 66.0606| 3.3602| 8.0756| 20.9| 804| 671| 1475| 11.0| 1.0| 1.0| 2.0| 1.0| 3.31| 0| 0| 0| 2.12| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.0964| 3.0000| 5.0886| 20.0| 768| 639| 1407| 8.0| 0.0| 0.0| 2.0| 1.0| 3.26| 0| 0| 0| 2.16| 0| 0
ti_estop| 15.09| 66.2487| 3.3298| 8.0818| 34.50| 1178| 934| 2112| 11.0| 1.0| 1.0| 4.0| 2.0| 3.26| 0| 0| 0| 4.48| 16.3| 0
ti_objdet_range| 15.12| 66.1525| 3.1031| 5.5799| 50.12| 1694| 1363| 3057| 8.0| 0.0| 0.0| 7.0| 3.0| 10.20| 0| 0| 0| 7.6| 24.81| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 15.64| 63.9216| 3.2961| 8.0810| 18.62| 571| 354| 925| 11.0| 10.0| 1.0| 2.0| 1.0| 3.28| 0| 0| 0| 2.18| 0| 0
ti_vision_cnn (objdet)| 14.84| 67.3990| 3.0000| 5.0942| 18.87| 579| 349| 928| 8.0| 10.0| 1.0| 2.0| 1.0| 3.29| 0| 0| 0| 2.18| 0| 0
