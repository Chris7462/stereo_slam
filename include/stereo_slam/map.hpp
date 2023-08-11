#pragma once

// c++ header
#include <memory>
#include <mutex>
#include <unordered_map>

// local header
#include "stereo_slam/common.hpp"
#include "stereo_slam/frame.hpp"
#include "stereo_slam/mappoint.hpp"


/**
 * @brief Map
 * Interaction with the map: the frontend calls InsertKeyframe and InsertMapPoint
 * to insert new frames and map points, the backend maintains the structure of
 * the map, determines outlier/culling, etc.
 */
class Map
{
  public:
    using Ptr = std::shared_ptr<Map>;
    using LandmarksType = std::unordered_map<size_t, MapPoint::Ptr>;
    using KeyframesType = std::unordered_map<size_t, Frame::Ptr>;

    Map() = default;

    /// Insert a key frame
    void InsertKeyFrame(Frame::Ptr frame);

    /// Insert a map point
    void InsertMapPoint(MapPoint::Ptr map_point);

    /// Get all map points
    LandmarksType GetAllMapPoints();

    /// Get all key frames
    KeyframesType GetAllKeyFrames();

    /// Get active map points
    LandmarksType GetActiveMapPoints();

    /// Get active key frames
    KeyframesType GetActiveKeyFrames();

    /// Clean up the points in map where the number of observations is zero
    void CleanMap();

    /// Clean up everything
    void Reset();

  private:
    // Deactive old key frame
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_; // all landmarks
    LandmarksType active_landmarks_; // active landmarks
    KeyframesType keyframes_; // all key-frames
    KeyframesType active_keyframes_; // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    size_t num_active_keyframes_ = 7; // number of activated keyframes
};
