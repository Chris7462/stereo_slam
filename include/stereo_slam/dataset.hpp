#pragma once

// c++ header
#include <memory>
#include <vector>
#include <queue>
#include <mutex>

// opencv header
#include <opencv2/core.hpp>

// local header
#include "stereo_slam/camera.hpp"
#include "stereo_slam/common.hpp"
#include "stereo_slam/frame.hpp"


class Dataset
{
  public:
    using Ptr = std::shared_ptr<Dataset>;
    Dataset(const std::string& dataset_path);

    bool Init();

    // create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

    std::queue<cv::Mat> imgLeftResized_;
    std::queue<cv::Mat> imgRightResized_;

    bool new_img_available_;
    std::mutex data_mutex_;

  private:
    std::string dataset_path_;
    int current_image_index_;

    std::vector<Camera::Ptr> cameras_;
};
