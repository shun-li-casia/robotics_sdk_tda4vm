# Performance Report

Performance statistics logging is turned on by setting a launch parameter, `exportPerfStats` to 1.

## TDA4VM

### ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on the target SK board. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.13| 66.1128| NA| NA| 18.38| 694| 528| 1222| 1.0| 0.0| 0.0| 3.0| 1.0| 0| 0| 0| 0| 4.52| 16.31| 0
ti_sde (w/ pcl)| 15.12| 66.1162| NA| NA| 33.25| 1051| 927| 1978| 0.0| 16.0| 0.0| 4.0| 3.0| 6.74| 0| 0| 0| 4.59| 21.10| 0
ti_vision_cnn (semseg)| 15.13| 66.0911| 3.0177| 8.0303| 11.97| 525| 309| 834| 11.0| 0.0| 0.0| 2.0| 1.0| 3.27| 0| 0| 0| 2.17| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.1066| 3.0051| 5.0025| 11.47| 491| 278| 769| 8.0| 1.0| 0.0| 2.0| 1.0| 3.27| 0| 0| 0| 2.16| 0| 0
ti_estop| 15.15| 66.0077| 3.0026| 8.0720| 25.62| 927| 603| 1530| 11.0| 0.0| 0.0| 4.0| 2.0| 3.29| 0| 0| 0| 4.52| 16.17| 0
ti_objdet_range| 15.15| 65.9896| 3.0000| 5.0415| 40.89| 1406| 1029| 2435| 8.0| 1.0| 0.0| 6.0| 3.0| 10.27| 0| 0| 0| 6.97| 24.83| 0
ti_vl| 15.14| 66.0387| 3.0193| 17.0386| 30.67| 1138| 976| 2114| 18.0| 68.0| 2.0| 2.0| 1.0| 1.25| 0| 0| 0| 0.79| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.11| 66.1753| NA| NA| 25.18| 923| 860| 1783| 1.0| 1.0| 0.0| 2.0| 2.0| 0| 0| 0| 0| 4.42| 16.46| 0
ti_sde (w/ pcl)| 15.12| 66.1377| NA| NA| 40.75| 1328| 1311| 2639| 0.0| 17.0| 0.0| 5.0| 3.0| 6.59| 0| 0| 0| 4.48| 21.32| 0
ti_vision_cnn (semseg)| 15.12| 66.1263| 3.1748| 8.0026| 20.63| 802| 671| 1473| 11.0| 0.0| 0.0| 2.0| 1.0| 3.23| 0| 0| 0| 2.15| 0| 0
ti_vision_cnn (objdet)| 15.14| 66.0699| 3.0000| 5.0080| 20.19| 761| 639| 1400| 8.0| 1.0| 1.0| 2.0| 1.0| 3.22| 0| 0| 0| 2.14| 0| 0
ti_estop| 15.14| 66.0397| 3.1372| 8.0871| 33.33| 1166| 936| 2102| 11.0| 0.0| 0.0| 4.0| 2.0| 3.34| 0| 0| 0| 4.59| 16.39| 0
ti_objdet_range| 15.11| 66.1702| 3.0522| 5.5039| 49.50| 1682| 1358| 3040| 8.0| 1.0| 0.0| 7.0| 3.0| 9.97| 0| 0| 0| 6.88| 25.11| 0

### Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.48| 32.8044| 3.4448| 8.0051| 45.5| 1247| 772| 2019| 22.0| 20.0| 0.0| 4.0| 1.0| 6.47| 0| 0| 0| 4.29| 0| 0
ti_vision_cnn (objdet)| 30.46| 32.8274| 3.0379| 5.0483| 36.22| 1092| 664| 1756| 15.0| 20.0| 0.0| 4.0| 1.0| 6.42| 0| 0| 0| 4.26| 0| 0

## AM68A

### ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on the target SK board. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1523| NA| NA| 18.25| 1216| 546| 1762| 0.0| 0| 0| 2.0| 1.0| 0| 0| 0| 0| 4.27| 16.10| 0
ti_sde (w/ pcl)| 15.12| 66.1279| NA| NA| 33.24| 1606| 912| 2518| 0.0| 0| 0| 3.0| 2.0| 6.26| 0| 0| 0| 4.27| 21.29| 0
ti_vision_cnn (semseg)| 15.12| 66.1458| 3.0102| 10.0102| 12.96| 1836| 1025| 2861| 15.0| 0| 0| 2.0| 1.0| 3.16| 0| 0| 0| 2.10| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.1079| 3.0179| 5.0026| 10.57| 1111| 387| 1498| 7.0| 0| 0| 1.0| 1.0| 3.9| 0| 0| 0| 2.6| 0| 0
ti_estop| 15.15| 66.0026| 3.0182| 10.0961| 25.18| 2267| 1333| 3600| 15.0| 0| 0| 2.0| 1.0| 3.10| 0| 0| 0| 4.24| 16.44| 0
ti_objdet_range| 13.12| 76.2331| 3.0092| 5.0245| 40.35| 2077| 1180| 3257| 7.0| 0| 0| 4.0| 2.0| 9.60| 0| 0| 0| 6.45| 24.95| 0
ti_vl| 15.14| 66.0396| 3.0395| 13.2006| 30.15| 1957| 1272| 3229| 13.0| 0| 0| 1.0| 1.0| 1.11| 0| 0| 0| 0.71| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.

### Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.11| 66.1728| NA| NA| 28.53| 1464| 881| 2345| 0.0| 0| 0| 2.0| 1.0| 0| 0| 0| 0| 4.26| 16.9| 0
ti_sde (w/ pcl)| 15.12| 66.1395| NA| NA| 39.84| 1896| 1300| 3196| 0.0| 0| 0| 3.0| 2.0| 6.38| 0| 0| 0| 4.34| 20.98| 0
ti_vision_cnn (semseg)| 15.12| 66.1269| 3.4677| 10.0258| 18.32| 2119| 1388| 3507| 15.0| 0| 0| 1.0| 1.0| 3.12| 0| 0| 0| 2.8| 0| 0
ti_vision_cnn (objdet)| 15.12| 66.1273| 3.0078| 5.0052| 17.67| 1400| 753| 2153| 7.0| 0| 0| 2.0| 1.0| 3.15| 0| 0| 0| 2.9| 0| 0
ti_estop| 15.15| 66.0272| 3.2636| 10.1005| 33.83| 2506| 1658| 4164| 15.0| 0| 0| 2.0| 1.0| 3.17| 0| 0| 0| 4.35| 16.27| 0
ti_objdet_range| 15.12| 66.1250| 3.1192| 5.1518| 48.73| 2361| 1501| 3862| 7.0| 0| 0| 5.0| 2.0| 9.65| 0| 0| 0| 6.75| 25.3| 0

### Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.50| 32.7890| 3.1703| 10.1007| 46.41| 3362| 2157| 5519| 30.0| 0| 0| 2.0| 1.0| 6.14| 0| 0| 0| 4.11| 0| 0
ti_vision_cnn (objdet)| 30.48| 32.8050| 3.0000| 5.0039| 42.43| 1845| 846| 2691| 13.0| 0| 0| 2.0| 1.0| 6.15| 0| 0| 0| 4.11| 0| 0

## AM69A

### ROSBAG, 15 FPS

**Source**: "rosbag play" and a demo ROS node are running in the ROS 1 Docker container on the target SK board. ROSBAG (zed1_2020-11-09-18-01-08.bag, 1280x720) is played back at 15 FPS.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.12| 66.1160| NA| NA| 4.30| 642| 522| 1164| 0.0| 0| 0| 2.0| 1.0| 0| 0| 0| 0| 4.28| 16.11| 0
ti_sde (w/ pcl)| 15.13| 66.0742| NA| NA| 8.24| 976| 901| 1877| 0.0| 0| 0| 3.0| 2.0| 6.27| 0| 0| 0| 4.27| 21.30| 0
ti_vision_cnn (semseg)| 15.12| 66.1432| 3.8547| 9.0025| 2.74| 1132| 1162| 2294| 13.0| 0| 0| 2.0| 1.0| 3.17| 0| 0| 0| 2.10| 0| 0
ti_vision_cnn (objdet)| 15.13| 66.1043| 3.1114| 4.0470| 2.61| 533| 376| 909| 7.0| 0| 0| 1.0| 1.0| 3.9| 0| 0| 0| 2.5| 0| 0
ti_estop| 15.15| 66.0227| 3.7708| 9.0730| 6.10| 1509| 1456| 2965| 13.0| 0| 0| 2.0| 1.0| 3.13| 0| 0| 0| 4.29| 16.2| 0
ti_objdet_range| 15.15| 66.0000| 3.0150| 4.0850| 9.26| 1372| 1134| 2506| 7.0| 0| 0| 4.0| 2.0| 9.62| 0| 0| 0| 6.54| 24.60| 0
ti_vl| 15.20| 65.7929| 3.0136| 16.3451| 8.67| 1344| 1225| 2569| 12.0| 0| 0| 2.0| 1.0| 1.10| 0| 0| 0| 0.70| 0| 0

**Note**: "A72 Load (%)" are for 8x A72 cores in a scale of 100%. For example, 100% A72 loading means that 8x A72 cores are fully loaded.

### Live ZED Camera, 15 FPS

**Source**: live ZED camera, 1280x720 on each of left and right image, at 15 FPS. "zed_capture" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_sde| 15.15| 66.0203| NA| NA| 5.8| 1316| 823| 2139| 0.0| 0| 0| 2.0| 1.0| 0| 0| 0| 0| 4.30| 16.13| 0
ti_sde (w/ pcl)| 15.14| 66.0584| NA| NA| 10.48| 1717| 1257| 2974| 0.0| 0| 0| 3.0| 3.0| 6.29| 0| 0| 0| 4.31| 20.68| 0
ti_vision_cnn (semseg)| 15.15| 66.0000| 4.0050| 9.0025| 4.36| 1850| 1493| 3343| 13.0| 0| 0| 2.0| 1.0| 3.10| 0| 0| 0| 2.7| 0| 0
ti_vision_cnn (objdet)| 15.16| 65.9825| 3.0000| 4.9950| 3.75| 1250| 708| 1958| 7.0| 0| 0| 1.0| 1.0| 3.12| 0| 0| 0| 2.8| 0| 0
ti_estop| 15.15| 66.0000| 4.0000| 9.0799| 7.76| 2190| 1758| 3948| 13.0| 0| 0| 2.0| 1.0| 3.14| 0| 0| 0| 4.33| 16.9| 0
ti_objdet_range| 15.16| 65.9599| 3.0000| 5.0000| 10.87| 2075| 1437| 3512| 7.0| 0| 0| 5.0| 2.0| 9.57| 0| 0| 0| 6.53| 24.44| 0

### Live C920 Webcam, 30 FPS

**Source**: live C920 webcam, 1280x720 in MJPG mode, at 30 FPS. "gscam" ROS node and a demo ROS node are running in the ROS 1 Docker container on the target SK board.

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (semseg)| 30.47| 32.8163| 4.0168| 9.0078| 14.4| 2926| 2419| 5345| 25.0| 0| 0| 3.0| 1.0| 6.24| 0| 0| 0| 4.32| 0| 0
ti_vision_cnn (objdet)| 30.48| 32.8105| 3.0050| 4.6974| 9.54| 1660| 820| 2480| 13.0| 0| 0| 2.0| 1.0| 6.23| 0| 0| 0| 4.37| 0| 0
