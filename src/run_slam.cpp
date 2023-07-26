// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <memory>
#include <thread>
#include <string>

// local header
#include "stereo_slam/slam_node.hpp"
#include "stereo_slam/visual_odometry.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::string config_file {"./src/stereo_slam/config/default.yaml"};
  auto vo_ptr {std::make_shared<VisualOdometry>(config_file)};

  auto slam_ptr {std::make_shared<SlamNode>(vo_ptr)};

  // start the SLAM thread
  std::thread slamthread(&VisualOdometry::Run, vo_ptr);
  std::thread image_processing(&SlamNode::image_processing, slam_ptr);

  rclcpp::spin(slam_ptr);
  rclcpp::shutdown();

  vo_ptr->Shutdown();

  return 0;
}
