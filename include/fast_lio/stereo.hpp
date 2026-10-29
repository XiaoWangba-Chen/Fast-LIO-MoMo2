#pragma once
#include "fast_lio/types.hpp"
#include <opencv2/core.hpp>
namespace fastlio {
class StereoTracker {
 public:
  int max_features=180;double min_disparity=1.,max_disparity=256.;
  StereoFrame track(double time,const cv::Mat& left,const cv::Mat& right);
  void reset();
 private:
  cv::Mat previous_;std::vector<cv::Point2f> points_;std::vector<uint64_t> ids_;
  uint64_t feature_id_=0,frame_id_=0;
};
}
