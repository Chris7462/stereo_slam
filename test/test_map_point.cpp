#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/mappoint.hpp"
#include "stereo_slam/feature.hpp"
#include "stereo_slam/frame.hpp"


#include <opencv2/core.hpp>


TEST(MapPoint, MapPoint_Pos_TC1_Test)
{
  Vec3 pos(1, 2, 3);
  MapPoint map_point(0, pos);
  Vec3 rtn = map_point.Pos();
  EXPECT_EQ(rtn(0), 1);
  EXPECT_EQ(rtn(1), 2);
  EXPECT_EQ(rtn(2), 3);
}

TEST(MapPoint, MapPoint_SetPos_TC1_Test)
{
  Vec3 init_pos(0, 0, 0);
  Vec3 set_pos(1, 2, 3);
  MapPoint map_point(0, init_pos);
  map_point.SetPos(set_pos);
  Vec3 rtn = map_point.Pos();
  EXPECT_EQ(rtn(0), 1);
  EXPECT_EQ(rtn(1), 2);
  EXPECT_EQ(rtn(2), 3);
}

TEST(MapPoint, MapPoint_AddObservation_TC1_Test)
{
  Vec3 pos(1, 2, 3);
  MapPoint map_point(0, pos);
  std::shared_ptr<Feature> f;
  map_point.AddObservation(f);
  EXPECT_EQ(map_point.observed_times_, 1);
  map_point.AddObservation(f);
  map_point.AddObservation(f);
  EXPECT_EQ(map_point.observed_times_, 3);
}

TEST(MapPoint, MapPoint_RemoveObservation_TC1_Test)
{
  Vec3 pos(1, 2, 3);
  MapPoint map_point(0, pos);
  cv::KeyPoint kp(cv::Point2f(0, 0), 2);
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  std::shared_ptr<Frame> frame_ptr = std::make_shared<Frame>(0, 100.0, pose, left, right);
  std::shared_ptr<Feature> feat = std::make_shared<Feature>(frame_ptr, kp);
  map_point.AddObservation(feat);
  map_point.RemoveObservation(feat);
  EXPECT_EQ(map_point.observed_times_, 0);
}

TEST(MapPoint, MapPoint_GetObs_TC1_Test)
{
  Vec3 pos(1, 2, 3);
  MapPoint map_point(0, pos);
  cv::KeyPoint kp(cv::Point2f(0, 0), 2);
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  std::shared_ptr<Frame> frame_ptr = std::make_shared<Frame>(0, 100.0, pose, left, right);
  std::shared_ptr<Feature> feat = std::make_shared<Feature>(frame_ptr, kp);
  map_point.AddObservation(feat);
  map_point.AddObservation(feat);
  std::list<std::weak_ptr<Feature>> rtn = map_point.GetObs();
  EXPECT_EQ(int(rtn.size()), 2);
}

TEST(MapPoint, MapPoint_CreateNewMapPoint_TC1_Test)
{
  MapPoint map_point;
  MapPoint::Ptr map_ptr = map_point.CreateNewMappoint();
  EXPECT_EQ(int(map_ptr->id_), 0);
}
