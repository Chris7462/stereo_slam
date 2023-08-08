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
#include "stereo_slam/viewer.hpp"


class VisualOdometry
{
  public:
    using Ptr = std::shared_ptr<VisualOdometry>;

    /// constructor
    VisualOdometry();

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init(const std::vector<std::vector<double>>& projections, double num_features, double num_features_init);

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
    Viewer::Ptr GetVisualizeData();

    bool exit_;

  private:
    Dataset::Ptr dataset_;
    Frontend::Ptr frontend_;
    Backend::Ptr backend_;
    Map::Ptr map_;
    Viewer::Ptr viewer_;
};
