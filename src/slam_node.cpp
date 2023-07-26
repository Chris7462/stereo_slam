// ros header
#include <cv_bridge/cv_bridge.h>

// c++ header
#include <chrono>
#include <thread>
#include <vector>

// local header
#include "stereo_slam/slam_node.hpp"
#include "stereo_slam/common.hpp"


void SlamNode::leftCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr leftCameraMsg)
{
  std::lock_guard<std::mutex> lck(img_mutex_);
  imgLeftBuf_.push(leftCameraMsg);
}

void SlamNode::rightCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr rightCameraMsg)
{
  std::lock_guard<std::mutex> lck(img_mutex_);
  imgRightBuf_.push(rightCameraMsg);
}

cv::Mat SlamNode::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg)
{
  cv_bridge::CvImageConstPtr ptr;
  try {
    ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exeception: %s", e.what());
  }
  return ptr->image;
}

SlamNode::SlamNode(VisualOdometry::Ptr& vo)
  : Node("slam_node"), pvo_{vo}
{
  //declare_parameter("left_proj", rclcpp::PARAMETER_DOUBLE_ARRAY);
  //declare_parameter("right_proj", rclcpp::PARAMETER_DOUBLE_ARRAY);
  //rclcpp::Parameter left_projection_param = get_parameter("left_proj");
  //rclcpp::Parameter right_projection_param = get_parameter("right_proj");
  //std::vector<double> left_projection = left_projection_param.as_double_array();
  //std::vector<double> right_projection = right_projection_param.as_double_array();

  pvo_->Init();

  subLeftImg_ = create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera_gray_left/image_raw", 100,
    std::bind(&SlamNode::leftCameraHandler, this, std::placeholders::_1));

  subRightImg_ = create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera_gray_right/image_raw", 100,
    std::bind(&SlamNode::rightCameraHandler, this, std::placeholders::_1));
}

void SlamNode::image_processing()
{
  // TODO: change to false when slam is done.
  while (1) {
    cv::Mat img_left, img_right;

    { // for scope lock
      std::lock_guard<std::mutex> lck(img_mutex_);

      if (!imgLeftBuf_.empty() && !imgRightBuf_.empty()) {
        double left_time = rclcpp::Time(imgLeftBuf_.front()->header.stamp).seconds();
        double right_time = rclcpp::Time(imgRightBuf_.front()->header.stamp).seconds();

        // sync tolerance
        if (left_time < right_time - 0.015) {
          imgLeftBuf_.pop();
          RCLCPP_INFO_STREAM(get_logger(), "Throw left image --- Sync error: " << (left_time - right_time));
        } else if (left_time > right_time + 0.015) {
          imgRightBuf_.pop();
          RCLCPP_INFO_STREAM(get_logger(), "Throw right image --- Sync error: " << (left_time - right_time));
        } else {
          RCLCPP_INFO(get_logger(), "Sync success!");
          img_left = getImageFromMsg(imgLeftBuf_.front());
          img_right = getImageFromMsg(imgRightBuf_.front());
          imgLeftBuf_.pop();
          imgRightBuf_.pop();

          if (!img_left.empty() && !img_right.empty()) {
            cv::Mat img_left_resized, img_right_resized;
            cv::resize(img_left, img_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
            cv::resize(img_right, img_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
            pvo_->PushData(img_left_resized, img_right_resized);
          }
        }
      }
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}
