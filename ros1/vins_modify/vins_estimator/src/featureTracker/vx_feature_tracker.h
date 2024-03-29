/*******************************************************************************
 *   Copyright (C) 2024 CASIA. All rights reserved.
 *
 *   @Filename: vx_feature_detector.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 01/31/2024
 *
 *   @Description:
 *
 *******************************************************************************/

#pragma once

#include "vx_feature_detector.h"

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <execinfo.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker {
public:
  FeatureTracker();
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
  trackImage(double _cur_time, const vx_image &_img,
             const vx_image & img1);
  void setMask();
  void readIntrinsicParameter(const vector<string> &calib_file);
  void showUndistortion(const string &name);
  void rejectWithF();
  void undistortedPoints();
  vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts,
                                     camodocal::CameraPtr cam);
  vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                  map<int, cv::Point2f> &cur_id_pts,
                                  map<int, cv::Point2f> &prev_id_pts);
  void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
                    vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
  void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                 vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts,
                 vector<cv::Point2f> &curRightPts,
                 map<int, cv::Point2f> &prevLeftPtsMap);
  void setPrediction(map<int, Eigen::Vector3d> &predictPts);
  double distance(cv::Point2f &pt1, cv::Point2f &pt2);
  void removeOutliers(set<int> &removePtsIds);
  cv::Mat getTrackImage();
  bool inBorder(const cv::Point2f &pt);

  int row, col;
  cv::Mat imTrack;
  cv::Mat mask;
  cv::Mat fisheye_mask;
  vx_image prev_img, cur_img;
  vx_context vx_context_;
  vector<cv::Point2f> n_pts;
  vector<vx_keypoint_t> predict_pts;
  vector<cv::Point2f> predict_pts_debug;
  vector<vx_keypoint_t> prev_pts, cur_pts, cur_right_pts;
  vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
  vector<cv::Point2f> pts_velocity, right_pts_velocity;
  vector<int> ids, ids_right;
  vector<int> track_cnt;
  map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
  map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
  map<int, cv::Point2f> prevLeftPtsMap;
  vector<camodocal::CameraPtr> m_camera;
  double cur_time;
  double prev_time;
  bool stereo_cam;
  int n_id;
  bool hasPrediction;
};
