// c++ header
#include <iostream>
#include <fstream>

// opencv header
#include <opencv2/opencv.hpp>

// sophus header
#include <sophus/se3.hpp>

// local header
#include "stereo_slam/dataset.hpp"


Dataset::Dataset(const std::string& dataset_path)
  : new_img_available_{false}, dataset_path_(dataset_path), current_image_index_(0), cameras_(std::vector<Camera::Ptr>())
{
}

bool Dataset::Init() {
  // read camera intrinsics and extrinsics
  std::ifstream fin(dataset_path_ + "/calib.txt");
  if (!fin) {
    std::cout << "cannot find " << dataset_path_ << "/calib.txt!";
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    char camera_name[3];
    for (int k = 0; k < 3; ++k) {
        fin >> camera_name[k];
    }
    double projection_data[12];
    for (int k = 0; k < 12; ++k) {
        fin >> projection_data[k];
    }
    Mat33 K;
    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[4], projection_data[5], projection_data[6],
        projection_data[8], projection_data[9], projection_data[10];
    Vec3 t;
    t << projection_data[3], projection_data[7], projection_data[11];
    t = K.inverse() * t;
    K = K * 0.5;
    Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                      t.norm(), Sophus::SE3d(Sophus::SO3d(), t)));
    cameras_.push_back(new_camera);
    std::cout << "Camera " << i << " extrinsics: " << t.transpose() << std::endl << std::flush;
  }
  fin.close();
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
    current_image_index_++;

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
