#include <vector>

#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <gtest/gtest.h>

#include "stereo_slam/common.hpp"
#include "stereo_slam/dataset.hpp"

#include <opencv2/core.hpp>


TEST(Dataset, Dataset_Init_TC1_Test)
{
  Dataset dataset;
  std::vector<std::vector<double>> projection{
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1},
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1}
  };
  bool rtn = dataset.Init(projection);
  ASSERT_TRUE(rtn);
}

TEST(Dataset, Dataset_NextFrame_TC1_Test)
{
  Dataset dataset;
  std::vector<std::vector<double>> projection{
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1},
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1}
  };
  dataset.Init(projection);
  Frame::Ptr f;
  f = dataset.NextFrame();
  ASSERT_TRUE(f == nullptr);
  dataset.new_img_available_ = true;
  cv::Mat img;
  dataset.imgLeftResized_.push(img);
  dataset.imgRightResized_.push(img);

  f = dataset.NextFrame();
  EXPECT_FALSE(f == nullptr);
}

TEST(Dataset, Dataset_GetCamera_TC1_TEST)
{
  Dataset dataset;
  std::vector<std::vector<double>> projection{
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1},
    {1, 2, 3, 0, 4, 5, 6, 0, 7, 8, 9, 1}
  };
  dataset.Init(projection);
  Camera::Ptr camera = dataset.GetCamera(1);
  //TODO better checks
  ASSERT_FALSE(camera == nullptr);
}
