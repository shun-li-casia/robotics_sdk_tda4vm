Performance Report
==================

Performance statistics logging was turned on by setting `exportPerfStats: 1` in the application configuration .yaml file.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1465| NA| NA| 13.68| 604| 556| 1160| 1.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 5.0| 15.19| 0
ti_sde (w/ pcl)| 15.12| 66.1232| NA| NA| 24.37| 854| 812| 1666| 0.0| 16.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.95| 15.3| 0
ti_vision_cnn (semseg)| 15.12| 66.1227| 3.0153| 9.0336| 13.18| 606| 422| 1028| 10.0| 0.0| 1.0| 2.0| 1.0| 3.59| 0| 0| 0| 2.39| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.0722| 2.5306| 13.0510| 10.10| 779| 388| 1167| 18.0| 1.0| 0.0| 1.0| 1.0| 3.55| 0| 0| 0| 2.36| 0| 0
ti_estop| 15.13| 66.0851| 3.0133| 9.0239| 22.92| 974| 708| 1682| 10.0| 1.0| 1.0| 3.0| 1.0| 3.56| 0| 0| 0| 4.90| 15.24| 0
ti_vl| 15.15| 65.9883| 3.0145| 14.5710| 28.75| 967| 884| 1851| 16.0| 68.0| 2.0| 2.0| 1.0| 1.32| 0| 0| 0| 0.84| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.13| 66.1072| NA| NA| 20.30| 857| 880| 1737| 1.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.93| 14.98| 0
ti_sde (w/ pcl)| 15.11| 66.2010| NA| NA| 31.40| 1070| 1053| 2123| 1.0| 16.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.92| 14.92| 0
ti_vision_cnn (semseg)| 15.12| 66.1306| 3.0234| 9.0339| 19.84| 892| 784| 1676| 10.0| 0.0| 1.0| 2.0| 1.0| 3.59| 0| 0| 0| 2.38| 0| 0
ti_vision_cnn (objdet)| 15.12| 66.1194| 2.9412| 13.0147| 30.15| 1117| 643| 1760| 12.0| 0.0| 1.0| 1.0| 1.0| 2.29| 0| 0| 0| 1.52| 0| 0
ti_estop| 15.11| 66.1662| 3.0163| 9.0489| 35.0| 1238| 1042| 2280| 10.0| 0.0| 0.0| 2.0| 1.0| 3.64| 0| 0| 0| 5.0| 15.6| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.55| 32.7382| 3.0844| 9.7727| 43.45| 1363| 1040| 2403| 20.0| 21.0| 0.0| 3.0| 1.0| 7.11| 0| 0| 0| 4.73| 0| 0
ti_vision_cnn (objdet)| 30.54| 32.7486| 2.8370| 13.0543| 38.90| 1726| 924| 2650| 37.0| 21.0| 0.0| 3.0| 1.0| 7.9| 0| 0| 0| 4.70| 0| 0
