Performance Report
==================

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1319| NA| NA| 19.80| 691| 529| 1220| 0.0| 1.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.51| 16.25| 0
ti_sde (w/ pcl)| 15.13| 66.1101| NA| NA| 32.83| 1053| 922| 1975| 0.0| 16.0| 1.0| 3.0| 2.0| 6.61| 0| 0| 0| 4.50| 20.89| 0
ti_vision_cnn (semseg)| 15.12| 66.1410| 3.0205| 7.0026| 13.8| 517| 308| 825| 11.0| 1.0| 1.0| 1.0| 1.0| 3.22| 0| 0| 0| 2.14| 0| 0
ti_vision_cnn (objdet)| 15.12| 66.1294| 2.9946| 12.0027| 11.47| 700| 298| 998| 17.0| 0.0| 0.0| 2.0| 1.0| 3.21| 0| 0| 0| 2.12| 0| 0
ti_estop| 15.13| 66.1005| 3.0154| 7.0154| 24.81| 912| 597| 1509| 11.0| 0.0| 1.0| 2.0| 1.0| 3.32| 0| 0| 0| 4.57| 16.33| 0
ti_objdet_range| 13.08| 76.4712| 3.1486| 12.2297| 37.53| 1353| 906| 2259| 15.0| 0.0| 0.0| 3.0| 2.0| 8.1| 0| 0| 0| 5.54| 20.39| 0
ti_vl| 15.17| 65.9252| 3.0304| 16.9310| 29.69| 1150| 987| 2137| 17.0| 68.0| 2.0| 2.0| 1.0| 1.26| 0| 0| 0| 0.80| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1330| NA| NA| 24.18| 932| 864| 1796| 0.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.53| 16.34| 0
ti_sde (w/ pcl)| 15.12| 66.1205| NA| NA| 36.69| 1249| 1135| 2384| 0.0| 16.0| 1.0| 3.0| 2.0| 6.68| 0| 0| 0| 4.55| 20.97| 0
ti_vision_cnn (semseg)| 15.13| 66.1094| 3.3122| 7.0102| 20.78| 793| 669| 1462| 11.0| 1.0| 0.0| 2.0| 1.0| 3.27| 0| 0| 0| 2.17| 0| 0
ti_vision_cnn (objdet)| 15.12| 66.1266| 3.0000| 12.0026| 18.25| 971| 659| 1630| 17.0| 1.0| 1.0| 2.0| 1.0| 3.26| 0| 0| 0| 2.16| 0| 0
ti_estop| 15.12| 66.1221| 3.4683| 7.0132| 34.33| 1163| 932| 2095| 11.0| 1.0| 0.0| 2.0| 1.0| 3.30| 0| 0| 0| 4.54| 16.25| 0
ti_objdet_range| 15.12| 66.1429| 3.1135| 12.1319| 49.87| 1895| 1376| 3271| 18.0| 0.0| 0.0| 4.0| 2.0| 10.4| 0| 0| 0| 7.3| 24.67| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 29.82| 33.5381| 3.3170| 7.2653| 35.56| 1153| 728| 1881| 20.0| 19.0| 1.0| 2.0| 1.0| 6.26| 0| 0| 0| 4.15| 0| 0
ti_vision_cnn (objdet)| 29.75| 33.6160| 3.0013| 12.0040| 36.18| 1514| 707| 2221| 34.0| 19.0| 0.0| 3.0| 1.0| 6.21| 0| 0| 0| 4.11| 0| 0
