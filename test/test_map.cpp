#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/map.hpp"

#include <opencv2/core.hpp>

TEST(Map, Map_InsertKeyFrame_TC1_Test)
{
  Map map;
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame::Ptr frame = std::make_shared<Frame>(0, 0.0, pose, left, right);
  map.InsertKeyFrame(frame);

  Map::KeyframesType rtn = map.GetAllKeyFrames();
  EXPECT_EQ(int(rtn.size()), 1);
}

TEST(Map, Map_InsertMapPoint_TC1_Test)
{
  Map map;
  Vec3 pos(1, 2, 3);
  MapPoint::Ptr mappoint = std::make_shared<MapPoint>(0, pos);
  map.InsertMapPoint(mappoint);

  Map::LandmarksType rtn = map.GetAllMapPoints();
  EXPECT_EQ(int(rtn.size()), 1);
}

TEST(Map, Map_GetActiveKeyFrames_TC1_Test)
{
  Map map;
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame::Ptr frame = std::make_shared<Frame>(0, 0, pose, left, right);
  frame->SetKeyFrame();
  map.InsertKeyFrame(frame);
  for (int i = 0; i < 8; ++i) {
    Frame::Ptr frame = std::make_shared<Frame>(size_t(i), double(i), pose, left, right);
    frame->SetKeyFrame();
    map.InsertKeyFrame(frame);
  }

  Map::KeyframesType rtn = map.GetActiveKeyFrames();
  EXPECT_EQ(int(rtn.size()), 7);
}

TEST(Map, Map_GetActiveMapPoints_TC1_Test)
{
  Map map;
  Vec3 pos(1, 2, 3);
  MapPoint::Ptr mappoint = std::make_shared<MapPoint>(0, pos);
  map.InsertMapPoint(mappoint);
  for (int i = 0; i < 8; ++i) {
    Vec3 pos(1, 2, i);
    MapPoint::Ptr mappoint = std::make_shared<MapPoint>(i, pos);
    map.InsertMapPoint(mappoint);
  }
  map.CleanMap();
  Map::LandmarksType rtn1 = map.GetAllMapPoints();
  Map::LandmarksType rtn2 = map.GetActiveMapPoints();
  EXPECT_EQ(int(rtn1.size()), 8);
  EXPECT_EQ(int(rtn2.size()), 0);
}

TEST(Map, Map_Reset_TC1_Test)
{
  Map map;
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  for (int i = 0; i < 8; ++i) {
    Frame::Ptr frame = std::make_shared<Frame>(size_t(i), double(i), pose, left, right);
    frame->SetKeyFrame();
    map.InsertKeyFrame(frame);
    Vec3 pos(1, 2, i);
    MapPoint::Ptr mappoint = std::make_shared<MapPoint>(i, pos);
    map.InsertMapPoint(mappoint);
  }

  map.Reset();
  Map::LandmarksType active_land = map.GetActiveMapPoints();
  Map::LandmarksType all_land = map.GetAllMapPoints();
  Map::KeyframesType active_kp = map.GetActiveKeyFrames();
  Map::KeyframesType all_kp = map.GetAllKeyFrames();
  EXPECT_EQ(int(active_land.size()), 0);
  EXPECT_EQ(int(all_land.size()), 0);
  EXPECT_EQ(int(all_kp.size()), 0);
  EXPECT_EQ(int(active_kp.size()), 0);
}
