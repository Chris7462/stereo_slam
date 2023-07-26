#pragma once

// c++ header
#include <memory>

// opencv header
#include <opencv2/features2d.hpp>

// local header
#include "stereo_slam/common.hpp"


class Frame;
class MapPoint;

/**
 * 2D feature point
 * will be associated to a map point after triangulation
 */
class Feature
{
  public:
    using Ptr = std::shared_ptr<Feature>;

    Feature() = default;

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& kp);

    std::weak_ptr<Frame> frame_; // feature frame
    cv::KeyPoint position_; // 2D position
    std::weak_ptr<MapPoint> map_point_; // associated map point

    bool is_outlier_ = false;
    bool is_on_left_image_ = true; // false means on the right image
};
