#pragma once

// c++ header
#include <memory>
#include <string>

// opencv header
#include <opencv2/core.hpp>

// local header
#include "stereo_slam/dataset.hpp"
#include "stereo_slam/frontend.hpp"
#include "stereo_slam/backend.hpp"


class VisualOdometry
{
  public:
    using Ptr = std::shared_ptr<VisualOdometry>;

    /// constructor with config file
    VisualOdometry(std::string& config_file);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    /**
     * Stop the VO
     */
    void Shutdown();

    /// Get frontend status
    FrontendStatus GetFrontendStatus() const;

    void PushData(cv::Mat& img_left_resize, cv::Mat& img_right_resize);

    bool exit_;

  private:
    std::string config_file_;

    Dataset::Ptr dataset_;
    Frontend::Ptr frontend_;
    Backend::Ptr backend_;
    Map::Ptr map_;
//  Viewer::Ptr viewer_;
};
