/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 * Modified by Shun Li
 *******************************************************/

#include "../estimator/estimator.h"
#include "../estimator/parameters.h"
#include "../utility/visualization.h"
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <stdio.h>
#include <thread>

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
std::mutex m_buf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  double t = imu_msg->header.stamp.toSec();
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Vector3d acc(dx, dy, dz);
  Vector3d gyr(rx, ry, rz);
  estimator.inputIMU(t, acc, gyr);
  return;
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg) {
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  for (unsigned int i = 0; i < feature_msg->points.size(); i++) {
    int feature_id = feature_msg->channels[0].values[i];
    int camera_id = feature_msg->channels[1].values[i];
    double x = feature_msg->points[i].x;
    double y = feature_msg->points[i].y;
    double z = feature_msg->points[i].z;
    double p_u = feature_msg->channels[2].values[i];
    double p_v = feature_msg->channels[3].values[i];
    double velocity_x = feature_msg->channels[4].values[i];
    double velocity_y = feature_msg->channels[5].values[i];
    if (feature_msg->channels.size() > 6) {
      double gx = feature_msg->channels[6].values[i];
      double gy = feature_msg->channels[7].values[i];
      double gz = feature_msg->channels[8].values[i];
      pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
      // printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
    }
    ROS_ASSERT(z == 1);
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }
  double t = feature_msg->header.stamp.toSec();
  estimator.inputFeature(t, featureFrame);
  return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg) {
  if (restart_msg->data == true) {
    ROS_WARN("restart the estimator!");
    estimator.clearState();
    estimator.setParameter();
  }
  return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg) {
  if (switch_msg->data == true) {
    // ROS_WARN("use IMU!");
    estimator.changeSensorType(1, STEREO);
  } else {
    // ROS_WARN("disable IMU!");
    estimator.changeSensorType(0, STEREO);
  }
  return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg) {
  if (switch_msg->data == true) {
    // ROS_WARN("use stereo!");
    estimator.changeSensorType(USE_IMU, 1);
  } else {
    // ROS_WARN("use mono camera (left)!");
    estimator.changeSensorType(USE_IMU, 0);
  }
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle n;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  if (argc != 2) {
    printf("please intput: rosrun vins vins_node [config file] \n"
           "for example: rosrun vins vins_node "
           "~/catkin_ws/src/VINS-Fusion/config/euroc/"
           "euroc_stereo_imu_config.yaml \n");
    return 1;
  }

  string config_file = argv[1];
  printf("config_file: %s\n", argv[1]);

  readParameters(config_file);
  estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
  ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

  ROS_WARN("waiting for image and imu...");

  registerPub(n);

  ros::Subscriber sub_imu;
  if (USE_IMU) {
    sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback,
                          ros::TransportHints().tcpNoDelay());
  }
  ros::Subscriber sub_feature =
      n.subscribe("/feature_tracker/feature", 2000, feature_callback);
  ros::Subscriber sub_restart =
      n.subscribe("/vins_restart", 100, restart_callback);
  ros::Subscriber sub_imu_switch =
      n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
  ros::Subscriber sub_cam_switch =
      n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
  ros::spin();

  return 0;
}

