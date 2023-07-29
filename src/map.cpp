// c++ header
#include <iostream>
#include <utility>

// local header
#include "stereo_slam/map.hpp"
#include "stereo_slam/feature.hpp"


void Map::InsertKeyFrame(Frame::Ptr frame)
{
  current_frame_ = frame;
  if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
    keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
  } else {
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;
  }

  if (active_keyframes_.size() > num_active_keyframes_) {
    RemoveOldKeyframe();
  }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point)
{
  if (landmarks_.find(map_point->id_) == landmarks_.end()) {
    landmarks_.insert(std::make_pair(map_point->id_, map_point));
    active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
  } else {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
  }
}

Map::LandmarksType Map::GetAllMapPoints()
{
  std::unique_lock<std::mutex> lck(data_mutex_);
  return landmarks_;
}

Map::KeyframesType Map::GetAllKeyFrames()
{
  std::unique_lock<std::mutex> lck(data_mutex_);
  return keyframes_;
}

Map::LandmarksType Map::GetActiveMapPoints()
{
  std::unique_lock<std::mutex> lck(data_mutex_);
  return active_landmarks_;
}

Map::KeyframesType Map::GetActiveKeyFrames()
{
  std::unique_lock<std::mutex> lck(data_mutex_);
  return active_keyframes_;
}

void Map::CleanMap()
{
  int cnt_landmark_removed = 0;
  for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
    if (iter->second->observed_times_ == 0) {
      iter = active_landmarks_.erase(iter);
      cnt_landmark_removed++;
    } else {
      ++iter;
    }
  }
  std::cout << "Removed " << cnt_landmark_removed << " active landmarks" << std::endl;
}

void Map::RemoveOldKeyframe()
{
  if (current_frame_ == nullptr) {
    return;
  }

  // Find the closest and farthest keyframes from the current frame
  double max_dis = 0.0;
  double min_dis = 9999.0;
  size_t max_kf_id = 0;
  size_t min_kf_id = 0;

  auto Twc = current_frame_->Pose().inverse();
  for (auto& kf: active_keyframes_) {
    if (kf.second == current_frame_) {
      continue;
    } else {
      auto dis = (kf.second->Pose() * Twc).log().norm();
      if (dis > max_dis) {
        max_dis = dis;
        max_kf_id = kf.first;
      }
      if (dis < min_dis) {
        min_dis = dis;
        min_kf_id = kf.first;
      }
    }
  }

  const double min_dis_th = 0.2; // minimum threshold
  Frame::Ptr frame_to_remove = nullptr;
  if (min_dis < min_dis_th) {
    // remove closest
    frame_to_remove = keyframes_.at(min_kf_id);
  } else {
    // remove farthest
    frame_to_remove = keyframes_.at(max_kf_id);
  }

  std::cout << "remove keyframe " << frame_to_remove->keyframe_id_ << std::endl;
  // remove keyframe and landmark observation
  active_keyframes_.erase(frame_to_remove->keyframe_id_);
  for (auto feat: frame_to_remove->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->RemoveObservation(feat);
    }
  }

  for (auto feat: frame_to_remove->features_right_) {
    if (feat == nullptr) {
      continue;
    } else {
      auto mp = feat->map_point_.lock();
      if (mp) {
        mp->RemoveObservation(feat);
      }
    }
  }

  CleanMap();
}
