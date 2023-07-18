// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <memory>
#include <thread>

// local header
#include "stereo_slam/slam_node.hpp"
#include "stereo_slam/visual_odometry.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // start the SLAM thread
  auto vo_ptr {std::make_shared<VisualOdometry>()};
  vo_ptr->Init();
  std::thread slamthread(&VisualOdometry::Run, *vo_ptr);

  auto slam_ptr {std::make_shared<SlamNode>()};
  std::thread image_processing(&SlamNode::image_processing, slam_ptr);
  rclcpp::spin(slam_ptr);
  rclcpp::shutdown();

  return 0;
}
