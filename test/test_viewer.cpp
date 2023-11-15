#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/viewer.hpp"

#include <opencv2/core.hpp>

TEST(Viewer, Viewer_AddCurrentFrame_TC1_Test)
{
  Viewer::Ptr viewer = Viewer::Ptr(new Viewer);
  Map::Ptr map = std::make_shared<Map>();
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame::Ptr frame = std::make_shared<Frame>(0, 0.0, pose, left, right);

  map->InsertKeyFrame(frame);
  viewer->Reset();
  viewer->SetMap(map);
  viewer->AddCurrentFrame(frame);
  viewer->UpdateMap();

  viewer->Close();
}
