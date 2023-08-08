#include "stereo_slam/frame.hpp"


Frame::Frame(size_t id, double time_stamp, const Sophus::SE3d& pose,
  const cv::Mat& left, const cv::Mat& right)
  : left_img_{left}, right_img_{right}, id_{id}, keyframe_id_{0},
    is_keyframe_{false}, time_stamp_{time_stamp}, pose_{pose}
{
}

Sophus::SE3d Frame::Pose()
{
  std::unique_lock<std::mutex> lck(pose_mutex_);
  return pose_;
}

void Frame::SetPose(const Sophus::SE3d& pose)
{
  std::unique_lock<std::mutex> lck(pose_mutex_);
  pose_ = pose;
}

void Frame::SetKeyFrame() {
  static size_t keyframe_factory_id = 0;
  is_keyframe_ = true;
  keyframe_id_ = keyframe_factory_id++;
}

Frame::Ptr Frame::CreateFrame()
{
  static size_t factory_id = 0;
  Frame::Ptr new_frame(new Frame);
  new_frame->id_ = factory_id++;
  return new_frame;
}
