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
 * Modify by Shun Li
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
FeatureTracker featureTracker;

uint64_t feature_cnt = 0;

ros::Publisher pub_image_track, pub_img;
void pubTrackImage(const cv::Mat &imgTrack, const double t) {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(t);
  sensor_msgs::ImagePtr imgTrackMsg =
      cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
  pub_image_track.publish(imgTrackMsg);
}

queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  m_buf.lock();
  img0_buf.push(img_msg);
  m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  m_buf.lock();
  img1_buf.push(img_msg);
  m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  cv::Mat img = ptr->image.clone();
  return img;
}

// extract images with same timestamp from two topics
void sync_process() {
  while (ros::ok()) {
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;

    if (STEREO) {
      cv::Mat image0, image1;
      std_msgs::Header header;
      double time = 0;
      m_buf.lock();
      if (!img0_buf.empty() && !img1_buf.empty()) {
        double time0 = img0_buf.front()->header.stamp.toSec();
        double time1 = img1_buf.front()->header.stamp.toSec();
        // 0.003s sync tolerance
        if (time0 < time1 - 0.003) {
          img0_buf.pop();
          printf("throw img0\n");
        } else if (time0 > time1 + 0.003) {
          img1_buf.pop();
          printf("throw img1\n");
        } else {
          time = img0_buf.front()->header.stamp.toSec();
          header = img0_buf.front()->header;
          image0 = getImageFromMsg(img0_buf.front());
          img0_buf.pop();
          image1 = getImageFromMsg(img1_buf.front());
          img1_buf.pop();
          // printf("find img0 and img1\n");
        }
      }
      m_buf.unlock();
      if (!image0.empty()) {
        // TODO: control the track rate
        featureFrame = featureTracker.trackImage(time, image0, image1);
        feature_cnt++;
      }
    } else {
      cv::Mat image;
      std_msgs::Header header;
      double time = 0;
      m_buf.lock();
      if (!img0_buf.empty()) {
        time = img0_buf.front()->header.stamp.toSec();
        header = img0_buf.front()->header;
        image = getImageFromMsg(img0_buf.front());
        img0_buf.pop();
      }
      m_buf.unlock();
      if (!image.empty()) {
        featureFrame = featureTracker.trackImage(time, image);
        feature_cnt++;
      }
    }

    if (!img0_buf.empty() && feature_cnt % 2 == 0) {
      // pub the feature...
      sensor_msgs::PointCloud features_to_pub;
      features_to_pub.header = img0_buf.front()->header;

      sensor_msgs::ChannelFloat32 id_of_point;
      sensor_msgs::ChannelFloat32 id_of_cam;
      sensor_msgs::ChannelFloat32 u_of_point;
      sensor_msgs::ChannelFloat32 v_of_point;
      sensor_msgs::ChannelFloat32 velocity_x_of_point;
      sensor_msgs::ChannelFloat32 velocity_y_of_point;

      for (auto &feature : featureFrame) {
        const int &feature_id = feature.first;
        const vector<pair<int, Eigen::Matrix<double, 7, 1>>> &observations =
            feature.second;
        for (size_t i = 0; i < observations.size(); ++i) {
          int camera_id = observations[i].first;
          const Eigen::Matrix<double, 7, 1> &f = observations[i].second;

          geometry_msgs::Point32 p;
          p.x = f(0, 0);
          p.y = f(1, 0);
          p.z = f(2, 0);
          features_to_pub.points.push_back(p);

          id_of_point.values.push_back(feature_id);
          id_of_cam.values.push_back(camera_id);
          u_of_point.values.push_back(f(3, 0));
          v_of_point.values.push_back(f(4, 0));
          velocity_x_of_point.values.push_back(f(5, 0));
          velocity_y_of_point.values.push_back(f(6, 0));
        }
      }

      features_to_pub.channels.push_back(id_of_point);
      features_to_pub.channels.push_back(id_of_cam);
      features_to_pub.channels.push_back(u_of_point);
      features_to_pub.channels.push_back(v_of_point);
      features_to_pub.channels.push_back(velocity_x_of_point);
      features_to_pub.channels.push_back(velocity_y_of_point);

      pub_img.publish(features_to_pub);

      // pub the track image
      pubTrackImage(featureTracker.getTrackImage(),
                    features_to_pub.header.stamp.toSec());
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle n("~");
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
  featureTracker.readIntrinsicParameter(CAM_NAMES);

  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
  pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

  ros::Subscriber sub_imu;
  ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
  ros::Subscriber sub_img1;
  if (STEREO) {
    sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
  }
  std::thread sync_thread{sync_process};
  ros::spin();

  return 0;
}
