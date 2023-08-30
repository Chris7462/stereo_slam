#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/algorithm.hpp"


TEST(Algorithm, Triangulation_TC1)
{
  Vec3 pt_world(30, 20, 10), pt_world_estimated;
  std::vector<Sophus::SE3d> poses {
    Sophus::SE3d(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
    Sophus::SE3d(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Vec3(0.0, -10.0, 0.0)),
    Sophus::SE3d(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), Vec3(0.0, 10.0, 0.0))
  };
  std::vector<Vec3> points;
  for (size_t i = 0; i < poses.size(); ++i) {
    Vec3 pc = poses[i] * pt_world;
    pc /= pc[2];
    points.push_back(pc);
  }

  EXPECT_TRUE(triangulation(poses, points, pt_world_estimated));
  EXPECT_NEAR(pt_world[0], pt_world_estimated[0], 0.01);
  EXPECT_NEAR(pt_world[1], pt_world_estimated[1], 0.01);
  EXPECT_NEAR(pt_world[2], pt_world_estimated[2], 0.01);
}

TEST(Algorithm, ToVec2_TC1)
{
  cv::Point2f pt(0.1F, 0.2F);
  Vec2 vpt(toVec2(pt));

  EXPECT_FLOAT_EQ(vpt(0), 0.1F);
  EXPECT_FLOAT_EQ(vpt(1), 0.2F);
}
