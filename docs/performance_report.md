Performance Report
==================
### Test Setup
* "rosbag play" and demo ROS node are running in the ROS 1 Docker container on TDA4.
* ROSBAG (zed1_2020-11-09-18-01-08.bag, 720p) is played back at 15 FPS.
* Performance statistics logging is turned on by setting `exportPerfStats: 1` in the application configuration .yaml file.

### Performance Summary

Demo | FPS| Total time (ms)| Preproc time (ms)| Inference time (ms)| A72 Load (%)| DDR Read BW (MB/s)| DDR Write BW (MB/s)| DDR Total BW (MB/s)| C71 Load (%)| C66_1 Load (%)| C66_2 Load (%)| MCU2_0 Load (%)| MCU2_1 Load (%)| MSC_0 (%)| MSC_1 (%)| VISS (%)| NF (%)| LDC (%)| SDE (%)| DOF (%)
----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
ti_vision_cnn (objdet)| 15.03| 66.5252| 2.1700| 13.0100| 10.83| 781| 369| 1150| 18.0| 1.0| 1.0| 2.0| 1.0| 3.59| 0| 0| 0| 2.39| 0| 0
ti_vision_cnn (semseg)| 15.12| 66.1354| 3.0521| 9.0583| 13.31| 641| 460| 1101| 10.0| 0.0| 1.0| 2.0| 1.0| 3.54| 0| 0| 0| 2.35| 0| 0
ti_sde| 15.11| 66.1679| NA| NA| 25.75| 885| 817| 1702| 1.0| 16.0| 0.0| 2.0| 1.0| 0| 0| 0| 0| 4.94| 15.1| 0
ti_estop| 15.13| 66.1008| 3.0136| 9.0272| 24.68| 995| 711| 1706| 10.0| 0.0| 0.0| 2.0| 1.0| 3.57| 0| 0| 0| 4.91| 14.85| 0
ti_vl| 15.16| 65.9469| 3.0353| 14.7324| 30.15| 1019| 897| 1916| 16.0| 69.0| 2.0| 2.0| 1.0| 1.34| 0| 0| 0| 0.85| 0| 0

**Note**: "A72 Load (%)" are for dual A72 cores in a scale of 100%. For example, 100% A72 loading means that two A72 cores are fully loaded.