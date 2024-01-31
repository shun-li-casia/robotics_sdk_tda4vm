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

#include "vx_feature_detector.h"
#include "VX/vx_api.h"
#include <assert.h>

#include <algorithm>

VxFeatureDetector::VxFeatureDetector() { context_ = vxCreateContext(); }

vx_status VxFeatureDetector::GoodFetureToTrack(
    vx_image img, vx_array *corners, int max_corners_var,
    double quality_level_var, double min_distance_var, vx_image mask,
    int block_size_var, double K_var, double harris_K_size) {
  vx_status status = 0;
  assert(quality_level_var > 0 && min_distance_var >= 0 &&
         max_corners_var >= 0);
  // TODO: assert the mask ...
  //

  vx_scalar min_distance =
      vxCreateScalar(context_, VX_TYPE_FLOAT32, &min_distance_var);
  vx_scalar strength_thresh = vxCreateScalar(context_, VX_TYPE_FLOAT32, &K_var);
  vx_scalar sensitivity = vxCreateScalar(context_, VX_TYPE_FLOAT32, &K_var);

  vx_size num_corners_var = 0;
  vx_scalar num_corners =
      vxCreateScalar(context_, VX_TYPE_SIZE, &num_corners_var);

  vx_array tmp_corners = vxCreateArray(context_, VX_TYPE_KEYPOINT, 1000);
  status = vxuHarrisCorners(context_, img, strength_thresh, min_distance,
                            sensitivity, 3, 3, tmp_corners, num_corners);

  // apply the mask
  vx_size stride = sizeof(vx_size);
  void *base = NULL;
  vx_map_id map_id;
  vxMapArrayRange(tmp_corners, 0, num_corners_var, &map_id, &stride, &base,
                  VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

  vx_rectangle_t mask_rect;
  vx_imagepatch_addressing_t mask_addr;
  vx_map_id map_id_mask;
  unsigned char *mask_ptr;
  vx_uint32 mask_width, mask_height, num_bytes;

  vxQueryImage(mask, VX_IMAGE_WIDTH, &mask_width, sizeof(vx_uint32));
  vxQueryImage(mask, VX_IMAGE_HEIGHT, &mask_height, sizeof(vx_uint32));

  mask_rect.start_x = 0;
  mask_rect.start_y = 0;
  mask_rect.end_x = mask_width;
  mask_rect.end_y = mask_height;

  status = vxMapImagePatch(mask, &mask_rect, 0, &map_id_mask, &mask_addr,
                           (void **)&mask_ptr, VX_READ_ONLY,
                           VX_MEMORY_TYPE_HOST, VX_NOGAP_X);


  std::vector<vx_keypoint_t> masked_kps;
  for (vx_size i = 0; i < num_corners_var; ++i) {
    vx_keypoint_t kp = vxArrayItem(vx_keypoint_t, num_corners_var, i, stride);

    unsigned char *ptr = mask_ptr + kp.y * mask_height + kp.x * mask_width;
    if (*ptr) { // mask is 1
      masked_kps.push_back(kp);
    }
  }


  // TODO: what if the kps is not enough?
  if(masked_kps.size() <= num_corners_var) {
  ConvertVec2VxArray(masked_kps, masked_kps.size(), corners);
  } else {
  std::sort(masked_kps.begin(), masked_kps.end(), [](const vx_keypoint_t& a,
                                                     const vx_keypoint_t&
                                                     b) {return a.strength > b.strength;});

  ConvertVec2VxArray(masked_kps, num_corners_var, corners);
  }

return status;
}
