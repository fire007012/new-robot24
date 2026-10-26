#pragma once
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <cmath>
namespace vision_pkg {
// RealSense/Astra publish millimetres; Fortress depth images use metres.
inline cv_bridge::CvImageConstPtr depthMillimetres(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
  if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) return cv_bridge::toCvShare(msg);
  if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) throw cv_bridge::Exception("depth must be 16UC1 or 32FC1");
  auto source = cv_bridge::toCvShare(msg);
  auto result = std::make_shared<cv_bridge::CvImage>();
  result->header=msg->header;result->encoding=sensor_msgs::image_encodings::TYPE_16UC1;
  result->image=cv::Mat::zeros(source->image.size(),CV_16UC1);
  for(int y=0;y<source->image.rows;++y) for(int x=0;x<source->image.cols;++x) {
    float value=source->image.at<float>(y,x);
    if(std::isfinite(value) && value>0 && value<65.535f) result->image.at<uint16_t>(y,x)=static_cast<uint16_t>(std::round(value*1000.f));
  }
  return result;
}
}
