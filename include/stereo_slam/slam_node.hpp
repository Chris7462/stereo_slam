#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// c++ header
#include <queue>
#include <mutex>
#include <memory>

// opencv header
#include <opencv2/core.hpp>

// local header
#include "stereo_slam/visual_odometry.hpp"
#include "stereo_slam/dataset.hpp"
#include "stereo_slam/viewer.hpp"


class SlamNode: public rclcpp::Node
{
  public:
    SlamNode(VisualOdometry::Ptr& vo);
    void image_processing();
    void pubVisualizeData();

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subLeftImg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRightImg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImg_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubFramePose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pubMapPoint_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void leftCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr leftCameraMsg);
    void rightCameraHandler(const sensor_msgs::msg::Image::ConstSharedPtr rightCameraMsg);
    nav_msgs::msg::Odometry sophusToMsg(const Sophus::SE3d& se3);
    sensor_msgs::msg::PointCloud eigenToMsg(const std::vector<Vec3>& points);
    cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg);
    void pubTF2(const Sophus::SE3d& se3);

    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf_;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> imgRightBuf_;
    std::mutex img_mutex_;

    Dataset::Ptr dataset_;
    Viewer::Ptr viewer_;

    VisualOdometry::Ptr pvo_;
};
