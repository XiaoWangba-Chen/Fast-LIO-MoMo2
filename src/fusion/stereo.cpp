#include "fast_lio/stereo.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
namespace fastlio {
void StereoTracker::reset(){previous_.release();points_.clear();ids_.clear();feature_id_=frame_id_=0;}
StereoFrame StereoTracker::track(double time,const cv::Mat& left,const cv::Mat& right){
  if(left.empty()||right.empty()||left.size()!=right.size()||left.type()!=CV_8UC1||right.type()!=CV_8UC1)throw std::invalid_argument("Stereo requires equal-size mono8 rectified images");
  StereoFrame frame;frame.time=time;frame.id=++frame_id_;
  std::vector<cv::Point2f> current;std::vector<uint64_t> ids;
  if(!previous_.empty()&&!points_.empty()){
    std::vector<cv::Point2f> next,back;std::vector<uchar> ok,back_ok;std::vector<float> error;
    cv::calcOpticalFlowPyrLK(previous_,left,points_,next,ok,error);
    cv::calcOpticalFlowPyrLK(left,previous_,next,back,back_ok,error);
    std::vector<cv::Point2f> a,b;std::vector<uint64_t> valid_ids;
    for(size_t i=0;i<points_.size();++i)if(ok[i]&&back_ok[i]&&cv::norm(points_[i]-back[i])<1.){a.push_back(points_[i]);b.push_back(next[i]);valid_ids.push_back(ids_[i]);}
    std::vector<uchar> inliers(a.size(),1);
    if(a.size()>=8)cv::findFundamentalMat(a,b,cv::FM_RANSAC,1.,.99,inliers);
    for(size_t i=0;i<a.size();++i)if(i<inliers.size()&&inliers[i]){current.push_back(b[i]);ids.push_back(valid_ids[i]);}
  }
  cv::Mat mask(left.size(),CV_8UC1,cv::Scalar(255));
  for(auto p:current)cv::circle(mask,p,12,cv::Scalar(0),-1);
  if(static_cast<int>(current.size())<max_features){std::vector<cv::Point2f> extra;cv::goodFeaturesToTrack(left,extra,max_features-current.size(),.01,12,mask);for(auto p:extra){current.push_back(p);ids.push_back(++feature_id_);}}
  std::vector<cv::Point2f> next,back;std::vector<uchar> ok,back_ok;std::vector<float> error;
  points_.clear();ids_.clear();
  if(!current.empty()){
    cv::calcOpticalFlowPyrLK(left,right,current,next,ok,error,cv::Size(31,31),4);
    cv::calcOpticalFlowPyrLK(right,left,next,back,back_ok,error,cv::Size(31,31),4);
    for(size_t i=0;i<current.size();++i){
      double disparity=current[i].x-next[i].x;
      if(!ok[i]||!back_ok[i]||cv::norm(current[i]-back[i])>1||std::abs(current[i].y-next[i].y)>1.5||disparity<min_disparity||disparity>max_disparity||next[i].x<2||current[i].x>=left.cols-2||current[i].y<2||current[i].y>=left.rows-2)continue;
      points_.push_back(current[i]);ids_.push_back(ids[i]);frame.pixels.push_back(Pixel{ids[i],Vec3(current[i].x,current[i].y,next[i].x)});
    }
  }
  previous_=left.clone();return frame;
}
}
