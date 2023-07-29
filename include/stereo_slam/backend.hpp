#pragma once

// c++ header
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

// local header
#include "stereo_slam/camera.hpp"
#include "stereo_slam/map.hpp"

class Map;

/**
 * Backend.
 * There is a separate optimization thread that starts the optimization when the Map is updated
 * Map update is triggered by Frontend
 */
class Backend
{
  public:
    using Ptr = std::shared_ptr<Backend>;

    /// Start the optimization thread in the constructor and hang it
    Backend();

    // Setup left and right camera to get the intrinsic values
    void SetCameras(Camera::Ptr left, Camera::Ptr right);

    /// Setup the Map
    void SetMap(Map::Ptr map);

    /// Update map and run optimization
    void UpdateMap();

    /// Stop backend
    void Stop();

  private:
    void BackendLoop();

    /// Optimization for given keyframes and landmarks
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    Map::Ptr map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_;
    Camera::Ptr cam_right_;
};
