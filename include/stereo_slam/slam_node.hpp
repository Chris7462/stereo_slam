#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// c++ header
#include <queue>
#include <mutex>

// opencv header
#include <opencv2/core.hpp>


class SlamNode: public rclcpp::Node
{
  public:
    SlamNode();
    void image_processing();

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subLeftImg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRightImg_;

    void leftCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr leftCameraMsg);
    void rightCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr rightCameraMsg);

    cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg);

    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf_;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgRightBuf_;
    std::mutex img_mutex_;
};
