// local header
#include "stereo_slam/feature.hpp"


Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& kp)
  : frame_(frame), position_(kp)
{
}
