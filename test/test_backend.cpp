#include <vector>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/backend.hpp"

#include <opencv2/core.hpp>

#include <unistd.h>

TEST(Backend, Backend_Init_TC1_Test)
{

  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);

  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera::Ptr cam1;
  Camera::Ptr cam2;
  Map::Ptr map = Map::Ptr(new Map);
  for (int i = 0; i < 8; ++i) {
    Frame::Ptr frame = std::make_shared<Frame>(size_t(i), double(i), pose, left, right);
    frame->SetKeyFrame();
    map->InsertKeyFrame(frame);
    Vec3 pos(1, 2, i);
    MapPoint::Ptr mappoint = std::make_shared<MapPoint>(i, pos);
    map->InsertMapPoint(mappoint);
  }
  Backend::Ptr backend = Backend::Ptr(new Backend);
  backend->SetMap(map);
  backend->SetCameras(cam1, cam2);
  backend->UpdateMap();
  backend->Stop();
}
