Performance Report
==================

Performance statistics logging was turned on by setting `exportPerfStats: 1` in the application configuration .yaml file.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1253| NA| NA| 13.50| 597| 548| 1145| 0.0| 1.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.92| 14.96| 0
ti_sde (w/ pcl)| 15.11| 66.1700| NA| NA| 23.61| 853| 814| 1667| 1.0| 16.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.94| 15.1| 0
ti_vision_cnn (semseg)| 15.12| 66.1276| 3.0104| 9.0545| 12.75| 605| 423| 1028| 10.0| 1.0| 0.0| 2.0| 1.0| 3.50| 0| 0| 0| 2.33| 0| 0
ti_vision_cnn (objdet)| 15.21| 65.7500| 2.5752| 13.0354| 10.97| 771| 367| 1138| 18.0| 1.0| 1.0| 2.0| 1.0| 3.55| 0| 0| 0| 2.36| 0| 0
ti_estop| 15.13| 66.1114| 3.0132| 9.0291| 22.53| 977| 707| 1684| 10.0| 0.0| 1.0| 2.0| 1.0| 3.54| 0| 0| 0| 4.87| 15.23| 0
ti_vl| 15.15| 66.0199| 3.0198| 14.9943| 28.32| 967| 882| 1849| 17.0| 67.0| 2.0| 2.0| 1.0| 1.34| 0| 0| 0| 0.85| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.13| 66.1117| NA| NA| 21.15| 863| 882| 1745| 0.0| 1.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.95| 15.7| 0
ti_sde (w/ pcl)| 15.11| 66.1785| NA| NA| 31.73| 1121| 1159| 2280| 0.0| 16.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.91| 14.94| 0
ti_vision_cnn (semseg)| 15.13| 66.0969| 3.0235| 9.0601| 20.10| 892| 784| 1676| 10.0| 0.0| 0.0| 2.0| 1.0| 3.53| 0| 0| 0| 2.35| 0| 0
ti_vision_cnn (objdet)| 15.11| 66.1667| 2.7313| 13.0149| 28.68| 1113| 643| 1756| 12.0| 1.0| 0.0| 1.0| 1.0| 2.27| 0| 0| 0| 1.51| 0| 0
ti_estop| 15.12| 66.1560| 3.0139| 9.0583| 32.30| 1237| 1041| 2278| 10.0| 0.0| 0.0| 2.0| 1.0| 3.62| 0| 0| 0| 4.97| 15.5| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.54| 32.7440| 3.0952| 9.6010| 40.51| 1360| 1035| 2395| 21.0| 21.0| 1.0| 3.0| 1.0| 7.9| 0| 0| 0| 4.71| 0| 0
ti_vision_cnn (objdet)| 30.55| 32.7299| 3.0330| 13.1981| 39.19| 1720| 927| 2647| 36.0| 22.0| 0.0| 2.0| 1.0| 7.11| 0| 0| 0| 4.72| 0| 0
