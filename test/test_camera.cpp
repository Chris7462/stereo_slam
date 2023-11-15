#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/camera.hpp"

#include <opencv2/core.hpp>


TEST(Camera, Camera_Pose_TC1_Test)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera cam(0, 0, 0, 0, 0, pose);

  Sophus::SE3d rtn_pos = cam.pose();
  EXPECT_EQ(rtn_pos.matrix()(12), 1);
}

TEST(Camera, Camera_K_TC1_Test)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera cam(1, 2, 3, 4, 0, pose);

  Mat33 rtn_K = cam.K();
  EXPECT_EQ(rtn_K(0), 1);
  EXPECT_EQ(rtn_K(4), 2);
  EXPECT_EQ(rtn_K(6), 3);
  EXPECT_EQ(rtn_K(7), 4);
}

TEST(Camera, Camera_WorldCamera_TC1_TEST)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera cam(1, 2, 3, 4, 0, pose);
  Eigen::Matrix3d R_2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  Eigen::Vector3d t_2(0, 0, 1);
  Sophus::SE3d T_c_w(R_2, t_2);

  Vec3 vec(1, 2, 3);
  Vec3 camera_coord = cam.world2camera(vec, T_c_w);
  Vec3 inv = cam.camera2world(camera_coord, T_c_w);
  std::cout << inv(0) << std::endl;
  // EXPECT_EQ(inv(0), 1);
  // EXPECT_EQ(inv(1), 2);
  // EXPECT_EQ(inv(2), 3);
  //TODO cannot use expect eq here??
}


TEST(Camera, Camera_PixelCamera_TC1_TEST)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera cam(1, 2, 3, 4, 0, pose);

  Vec3 vec(1, 2, 3);
  Vec2 pixel_coord = cam.camera2pixel(vec);
  Vec3 inv = cam.pixel2camera(pixel_coord, 3);
  std::cout << inv << std::endl;
  // EXPECT_EQ(inv(0), 1);
  // EXPECT_EQ(inv(1), 2);
  // EXPECT_EQ(inv(2), 3);
  //TODO cannot use expect eq here??
}

TEST(Camera, Camera_WorldPixel_TC1_TEST)
{
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Camera cam(1, 2, 3, 4, 0, pose);
  Eigen::Matrix3d R_2 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
  Eigen::Vector3d t_2(0, 0, 1);
  Sophus::SE3d T_c_w(R_2, t_2);

  Vec3 vec(1, 2, 3);
  Vec2 pixel_coord = cam.world2pixel(vec, T_c_w);
  Vec3 inv = cam.pixel2world(pixel_coord, T_c_w, 3);
  std::cout << inv << std::endl;
  // EXPECT_EQ(inv(0), 1);
  // EXPECT_EQ(inv(1), 2);
  // EXPECT_EQ(inv(2), 3);
  //TODO cannot use expect eq here??
}
