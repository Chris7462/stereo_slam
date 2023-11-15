#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/frame.hpp"

#include <opencv2/core.hpp>

TEST(Frame, Frame_Init_TC1_Test)
{
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame frame(0, 0.0, pose, left, right);
  EXPECT_EQ(int(frame.id_), 0);
}

TEST(Frame, Frame_Pose_TC1_Test)
{
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame frame(0, 0.0, pose, left, right);

  Sophus::SE3d rtn = frame.Pose();
  EXPECT_EQ(rtn.matrix()(12), 1);
}

TEST(Frame, Frame_Set_Pose_TC1_Test)
{
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame frame(0, 0.0, pose, left, right);

  Eigen::Matrix3d R_new = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t_new(10, 10, 10);
  Sophus::SE3d new_pose(R_new, t_new);

  frame.SetPose(new_pose);
  Sophus::SE3d rtn = frame.Pose();
  EXPECT_EQ(rtn.matrix()(12), 10);
}

TEST(Frame, Frame_SetKeyFrame_TC1_Test)
{
  cv::Mat left = cv::Mat::zeros(500, 500, CV_8UC3);
  cv::Mat right = cv::Mat::zeros(500, 500, CV_8UC3);
  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Vector3d t(1, 0, 0);
  Sophus::SE3d pose(R, t);
  Frame frame(0, 0.0, pose, left, right);

  frame.SetKeyFrame();
  EXPECT_EQ(int(frame.keyframe_id_), 0);
}

TEST(Frame, Frame_CreateFrame_TC1_Test)
{
  Frame frame;
  Frame::Ptr frame_ptr = frame.CreateFrame();

  EXPECT_EQ(int(frame_ptr->id_), 0);
}
