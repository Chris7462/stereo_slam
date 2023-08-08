#include <iostream>
#include <chrono>
#include <thread>

#include "stereo_slam/visual_odometry.hpp"


VisualOdometry::VisualOdometry()
  : exit_{false}, dataset_{nullptr},
    frontend_{nullptr}, backend_{nullptr}, map_{nullptr}
{
}

bool VisualOdometry::Init(const std::vector<std::vector<double>>& projections, double num_features, double num_features_init)
{

  // Read datata set from projections
  dataset_ = Dataset::Ptr(new Dataset);
  bool init_dataset = dataset_->Init(projections);
  std::cout << "Dataset inited? " << (init_dataset ? "True" : "False") << std::endl;

  // create components and links
  frontend_ = Frontend::Ptr(new Frontend(num_features, num_features_init));
  backend_ = Backend::Ptr(new Backend);
  map_ = Map::Ptr(new Map);
  viewer_ = Viewer::Ptr(new Viewer);

  frontend_->SetBackend(backend_);
  frontend_->SetMap(map_);
  frontend_->SetViewer(viewer_);
  frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  backend_->SetMap(map_);
  backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  viewer_->SetMap(map_);

  return true;
}

void VisualOdometry::Run()
{
  while (!exit_) {
    // if we have data in the queue
    if (dataset_->new_img_available_) {
      // std::cout << "VO is running" << std::endl;
      if (Step() == false) {
        continue;
      }
    } else {
      std::chrono::milliseconds dura(1);
      std::this_thread::sleep_for(dura);
    }
  }
}

void VisualOdometry::Shutdown()
{
  std::cout << "VO exit" << std::endl;
  exit_ = true;
  backend_->Stop();
  viewer_->Close();
}

FrontendStatus VisualOdometry::GetFrontendStatus() const
{
  return frontend_->GetStatus();
}

bool VisualOdometry::Step()
{
  Frame::Ptr new_frame = dataset_->NextFrame();
  if (new_frame == nullptr) {
    return false;
  } else {
    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->Addframe(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "VO cost time: " << time_used.count() << " seconds." << std::endl;
    return success;
  }
}

void VisualOdometry::PushData(cv::Mat& img_left_resize, cv::Mat& img_right_resize)
{
  std::lock_guard<std::mutex> lck(dataset_->data_mutex_);
  dataset_->imgLeftResized_.push(img_left_resize);
  dataset_->imgRightResized_.push(img_right_resize);
  dataset_->new_img_available_ = true;
}

Viewer::Ptr VisualOdometry::GetVisualizeData()
{
  return viewer_;
}
