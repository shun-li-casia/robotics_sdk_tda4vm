Performance Report
==================

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on TDA4. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.11| 66.1597| NA| NA| 13.25| 598| 551| 1149| 1.0| 1.0| 1.0| 2.0| 1.0| 0| 0| 0| 0| 4.90| 14.88| 0
ti_sde (w/ pcl)| 15.12| 66.1425| NA| NA| 30.73| 1005| 926| 1931| 0.0| 16.0| 0.0| 3.0| 2.0| 7.28| 0| 0| 0| 4.97| 19.32| 0
ti_vision_cnn (semseg)| 15.12| 66.1181| 3.0052| 7.0000| 10.47| 486| 306| 792| 11.0| 0.0| 1.0| 2.0| 1.0| 3.54| 0| 0| 0| 2.36| 0| 0
ti_vision_cnn (objdet)| 15.12| 66.1348| 3.0000| 12.0081| 9.77| 706| 297| 1003| 19.0| 1.0| 1.0| 2.0| 1.0| 3.55| 0| 0| 0| 2.37| 0| 0
ti_estop| 15.12| 66.1295| 3.0052| 7.0052| 21.1| 865| 596| 1461| 11.0| 0.0| 0.0| 3.0| 1.0| 3.60| 0| 0| 0| 4.96| 14.97| 0
ti_objdet_range| 12.55| 79.7021| 3.0070| 12.9296| 32.50| 1348| 981| 2329| 16.0| 0.0| 1.0| 4.0| 2.0| 9.44| 0| 0| 0| 6.44| 19.41| 0
ti_vl| 15.16| 65.9628| 3.0057| 16.3429| 29.17| 1137| 981| 2118| 18.0| 80.0| 2.0| 2.0| 1.0| 1.36| 0| 0| 0| 0.87| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

## Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.13| 66.0869| NA| NA| 21.5| 854| 879| 1733| 0.0| 0.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.91| 14.91| 0
ti_sde (w/ pcl)| 15.12| 66.1568| NA| NA| 40.39| 1321| 1344| 2665| 0.0| 16.0| 0.0| 3.0| 2.0| 7.27| 0| 0| 0| 4.96| 19.30| 0
ti_vision_cnn (semseg)| 15.09| 66.2538| 3.2810| 7.0076| 17.46| 773| 669| 1442| 11.0| 0.0| 0.0| 2.0| 1.0| 3.52| 0| 0| 0| 2.35| 0| 0
ti_vision_cnn (objdet)| 15.14| 66.0517| 3.0026| 12.0747| 16.66| 989| 658| 1647| 19.0| 0.0| 0.0| 2.0| 1.0| 3.62| 0| 0| 0| 2.42| 0| 0
ti_estop| 15.13| 66.0968| 3.1930| 7.0080| 30.71| 1120| 928| 2048| 11.0| 0.0| 0.0| 2.0| 1.0| 3.59| 0| 0| 0| 4.96| 14.95| 0
ti_objdet_range| 15.11| 66.1645| 3.0032| 12.8360| 45.92| 1833| 1399| 3232| 19.0| 0.0| 0.0| 4.0| 2.0| 11.18| 0| 0| 0| 7.65| 22.97| 0

## Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on TDA4.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.54| 32.7405| 3.0260| 7.0977| 37.37| 1121| 802| 1923| 21.0| 21.0| 0.0| 3.0| 1.0| 7.12| 0| 0| 0| 4.76| 0| 0
ti_vision_cnn (objdet)| 30.60| 32.6822| 3.0000| 12.9290| 37.50| 1567| 780| 2347| 37.0| 21.0| 0.0| 3.0| 1.0| 7.6| 0| 0| 0| 4.71| 0| 0
