#pragma once

// c++ header
#include <thread>
#include <queue>
#include <vector>

// local header
#include "stereo_slam/common.hpp"
#include "stereo_slam/frame.hpp"
#include "stereo_slam/map.hpp"

/**
 * Visualizer
 */
class Viewer
{
  public:
    using Ptr = std::shared_ptr<Viewer>;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map; }

    void Close();

    // Add a current frame
    void AddCurrentFrame(Frame::Ptr current_frame);

    // Update the map
    void UpdateMap();

    // output buffer for ros2
    std::queue<Sophus::SE3d> pose_out_buf_;
    std::queue<cv::Mat> img_out_buf_;
    std::queue<std::vector<Vec3>> mappoint_out_buf_;

  private:
    void ThreadLoop();

    /// plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;
    bool new_frame_added_ = false;
    bool map_updated_ = false;

    std::unordered_map<size_t, Frame::Ptr> active_keyframes_;
    std::unordered_map<size_t, MapPoint::Ptr> active_landmarks_;

    std::mutex viewer_data_mutex_;
};
