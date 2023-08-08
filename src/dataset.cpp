// c++ header
#include <iostream>
#include <fstream>

// opencv header
#include <opencv2/opencv.hpp>

// sophus header
#include <sophus/se3.hpp>

// local header
#include "stereo_slam/dataset.hpp"


Dataset::Dataset()
  : new_img_available_{false}, current_image_index_(0), cameras_(std::vector<Camera::Ptr>())
{
}

bool Dataset::Init(const std::vector<std::vector<double>>& projections)
{
  int i = 0;
  // read in camera info
  for (auto& projection: projections) {
    Mat33 K;
    K << projection[0], projection[1], projection[2],
         projection[4], projection[5], projection[6],
         projection[8], projection[9], projection[10];
    Vec3 t;
    t << projection[3], projection[7], projection[11];
    t = K.inverse() * t;
    K = K * 0.5;
    Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                      t.norm(), Sophus::SE3d(Sophus::SO3d(), t)));
    cameras_.push_back(new_camera);
    std::cout << "Camera " << i << " extrinsics: " << t.transpose() << std::endl << std::flush;
    ++i;
  }

  current_image_index_ = 0;
  return true;
}

Frame::Ptr Dataset::NextFrame()
{
  if (new_img_available_) {
    // read images
    std::lock_guard<std::mutex> lck(data_mutex_);
    cv::Mat image_left, image_right;
    image_left = imgLeftResized_.front();
    image_right = imgRightResized_.front();
    imgLeftResized_.pop();
    imgRightResized_.pop();

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left;
    new_frame->right_img_ = image_right;
    ++current_image_index_;

    // sanity check
    if (imgLeftResized_.empty() || imgRightResized_.empty()) {
      new_img_available_ = false;
    }

    return new_frame;

  } else {
    std::cout << "No new images are available" << std::endl;
    std::cout << "Current image index is: " << current_image_index_ << std::endl;

    return nullptr;
  }
}
