//opencv header
#include <opencv2/opencv.hpp>

// local header
#include "stereo_slam/viewer.hpp"
#include "stereo_slam/feature.hpp"
#include "stereo_slam/frame.hpp"


Viewer::Viewer()
{
  viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close()
{
  viewer_running_ = false;
  viewer_thread_.join();
}

void Viewer::Reset()
{
  std::lock_guard<std::mutex> lck(viewer_data_mutex_);
  current_frame_ = nullptr;

  std::queue<Sophus::SE3d> pose_empty;
  std::swap(pose_out_buf_, pose_empty);

  std::queue<cv::Mat> img_empty;
  std::swap(img_out_buf_, img_empty);

  std::queue<std::vector<Vec3>> mappoint_empty;
  std::swap(mappoint_out_buf_, mappoint_empty);

  new_frame_added_ = false;
  map_updated_ = false;

  active_keyframes_.clear();
  active_landmarks_.clear();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
{
  std::lock_guard<std::mutex> lck(viewer_data_mutex_);
  current_frame_ = current_frame;
  new_frame_added_ = true;
}

void Viewer::UpdateMap()
{
  std::lock_guard<std::mutex> lck(viewer_data_mutex_);
  assert(map_ != nullptr);
  active_keyframes_ = map_->GetActiveKeyFrames();
  active_landmarks_ = map_->GetActiveMapPoints();
  map_updated_ = true;
}

void Viewer::ThreadLoop()
{
  while (viewer_running_) {
    if (new_frame_added_ && current_frame_) {
      std::lock_guard<std::mutex> lock(viewer_data_mutex_);
      cv::Mat img = PlotFrameImage();  //image for publishing
      img_out_buf_.push(img);
      pose_out_buf_.push(current_frame_->Pose().inverse());
      new_frame_added_ = false;
    }

    if (map_ && map_updated_) {
      std::lock_guard<std::mutex> lock(viewer_data_mutex_);
      std::vector<Vec3> map_points;
      for (auto& landmark: active_landmarks_) {
        auto pos = landmark.second->Pos();
        map_points.push_back(pos);
      }
      mappoint_out_buf_.push(map_points);
      map_updated_ = false;
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }

  std::cout << "Stop viewer" << std::endl;
}

cv::Mat Viewer::PlotFrameImage()
{
  cv::Mat img_out;
  cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.lock()) {
      auto feat = current_frame_->features_left_[i];
      cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0), 2);
    }
  }
  return img_out;
}
