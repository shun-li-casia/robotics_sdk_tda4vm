Performance Report
==================

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.13| 66.1005| NA| NA| 20.4| 692| 531| 1223| 1.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.95| 15.6| 0
ti_sde (w/ pcl)| 15.13| 66.0957| NA| NA| 33.83| 1061| 929| 1990| 0.0| 16.0| 0.0| 3.0| 2.0| 7.33| 0| 0| 0| 4.98| 19.42| 0
ti_vision_cnn (semseg)| 15.12| 66.1292| 3.0284| 7.0180| 11.80| 526| 309| 835| 11.0| 0.0| 1.0| 2.0| 1.0| 3.55| 0| 0| 0| 2.36| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.0788| 2.9973| 12.0027| 11.5| 699| 298| 997| 17.0| 1.0| 1.0| 2.0| 1.0| 3.56| 0| 0| 0| 2.36| 0| 0
ti_estop| 15.12| 66.1198| 3.0104| 7.0078| 26.55| 923| 600| 1523| 11.0| 0.0| 0.0| 2.0| 1.0| 3.63| 0| 0| 0| 4.99| 15.6| 0
ti_objdet_range| 12.12| 82.4800| 3.0145| 12.0761| 37.50| 1343| 886| 2229| 14.0| 1.0| 1.0| 3.0| 2.0| 8.66| 0| 0| 0| 5.96| 17.67| 0
ti_vl| 15.16| 65.9468| 3.0335| 16.7151| 31.92| 1154| 991| 2145| 18.0| 68.0| 2.0| 2.0| 1.0| 1.36| 0| 0| 0| 0.83| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1421| NA| NA| 24.68| 934| 861| 1795| 0.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.94| 15.5| 0
ti_sde (w/ pcl)| 15.13| 66.1068| NA| NA| 34.51| 1243| 1148| 2391| 0.0| 16.0| 0.0| 3.0| 2.0| 7.33| 0| 0| 0| 4.99| 19.43| 0
ti_vision_cnn (semseg)| 15.13| 66.0893| 3.4631| 7.0204| 17.81| 797| 670| 1467| 11.0| 1.0| 0.0| 2.0| 1.0| 3.52| 0| 0| 0| 2.33| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.0784| 3.0052| 12.0026| 18.20| 967| 659| 1626| 17.0| 1.0| 1.0| 2.0| 1.0| 3.60| 0| 0| 0| 2.38| 0| 0
ti_estop| 15.13| 66.1096| 3.4907| 7.0133| 32.99| 1162| 934| 2096| 11.0| 0.0| 1.0| 2.0| 1.0| 3.65| 0| 0| 0| 5.1| 15.13| 0
ti_objdet_range| 15.12| 66.1547| 3.0798| 12.1489| 48.98| 1884| 1373| 3257| 18.0| 1.0| 0.0| 4.0| 2.0| 11.6| 0| 0| 0| 7.68| 22.78| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 29.79| 33.5664| 3.0635| 7.1693| 31.75| 1079| 684| 1763| 20.0| 19.0| 1.0| 3.0| 1.0| 6.85| 0| 0| 0| 4.55| 0| 0
ti_vision_cnn (objdet)| 29.73| 33.6310| 3.0055| 12.0082| 36.76| 1548| 727| 2275| 35.0| 20.0| 0.0| 2.0| 1.0| 7.12| 0| 0| 0| 4.72| 0| 0
