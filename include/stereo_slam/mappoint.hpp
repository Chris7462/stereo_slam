#pragma once

// c++ header
#include <mutex>
#include <list>

// local header
#include "stereo_slam/common.hpp"


class Frame;
class Feature;

class MapPoint
{
  public:
    using Ptr = std::shared_ptr<MapPoint>;
    MapPoint() = default;

    MapPoint(size_t id, Vec3 position);

    Vec3 Pos();

    void SetPos(const Vec3& pos);

    void AddObservation(std::shared_ptr<Feature> feature);

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs();

    // factory function
    static MapPoint::Ptr CreateNewMappoint();

    Vec3 pos_ = Vec3::Zero();  // Position in world

    size_t id_ = 2;  // ID
    int observed_times_ = 0;  // being observed by feature matching algo.

    bool is_outlier_ = false;

  private:
    std::mutex data_mutex_;
    std::list<std::weak_ptr<Feature>> observations_;
};
