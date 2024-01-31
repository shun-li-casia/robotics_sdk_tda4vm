/*******************************************************************************
 *   Copyright (C) 2023 CASIA. All rights reserved.
 *
 *   @Filename: vx_feature_detector.h
 *
 *   @Author: shun li
 *
 *   @Email: shun.li.at.casia@outlook.com
 *
 *   @Date: 09/10/2023
 *
 *   @Description:
 *
 *******************************************************************************/

#pragma once

#include <VX/vx.h>
#include <VX/vxu.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

class VxFeatureDetector {
public:
  VxFeatureDetector();
  ~VxFeatureDetector();

  /* use the harrias corner to detect the good features to track */
  vx_status GoodFetureToTrack(vx_image img, vx_array *corners,
                              int max_corners_var, double quality_level_var,
                              double min_distance_var, vx_image mask,
                              int block_size_var, double K_var,
                              double harris_K_size);

private:
  vx_context context_;

  inline void ErrorCheck(vx_context *context_p, vx_status status,
                         const char *message) {
    if (status) {
      puts("ERROR! \n");
      puts(message);
      vxReleaseContext(context_p);
      exit(1);
    }
  }

  inline void ConvertVec2VxArray(const std::vector<vx_keypoint_t>& vec, int length, vx_array* arr) {
    int size = std::min(static_cast<int>(vec.size()), length);

    for(int i = 0; i < size; ++i) {
      const vx_keypoint_t& kp = vec[i];
      vxAddArrayItems(*arr, 1, &kp, sizeof(kp));
    }
  }
};
