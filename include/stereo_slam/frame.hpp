#pragma once

// c++ header
#include <vector>
#include <memory>
#include <mutex>

// opencv header
#include <opencv2/core.hpp>

// sophus
#include <sophus/se3.hpp>

// local header
#include "stereo_slam/camera.hpp"


// forward declare
class MapPoint;
class Feature;

class Frame
{
  public:  // data members
    using Ptr = std::shared_ptr<Frame>;
    Frame() = default;

    Frame(size_t id, double time_stamp, const Sophus::SE3d& pose,
          const cv::Mat& left, const cv::Mat& right);

    // set and get pose, thread safe
    Sophus::SE3d Pose();

    void SetPose(const Sophus::SE3d& pose);
    void SetKeyFrame();

    static Ptr CreateFrame();

    cv::Mat left_img_, right_img_;   // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

    size_t id_; // id of this frame
    size_t keyframe_id_;

  private:
    bool is_keyframe_;
    double time_stamp_;
    Sophus::SE3d pose_; // Tcw format
    std::mutex pose_mutex_;
};
