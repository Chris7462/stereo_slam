// ros header
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// c++ header
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

// Eigen header
#include <Eigen/Core>

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

cv::Mat SlamNode::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr imgMsg)
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
  : Node("slam_node"), pvo_(vo), gps_imu_init_(false), sync_(policy_t(10), subIMU_, subGPS_)
{
  std::vector<std::vector<double>> projections;
  for (int i = 0; i < 2; ++i) {
    std::string camera_name = "camera" + std::to_string(i);
    declare_parameter(camera_name, rclcpp::PARAMETER_DOUBLE_ARRAY);
    std::vector<double> projection = get_parameter(camera_name).as_double_array();
    projections.push_back(projection);
  }

  declare_parameter("num_features", rclcpp::PARAMETER_INTEGER);
  declare_parameter("num_features_init", rclcpp::PARAMETER_INTEGER);
  double num_features = get_parameter("num_features").as_int();
  double num_features_init = get_parameter("num_features_init").as_int();

  pvo_->Init(projections, num_features, num_features_init);
  viewer_ = pvo_->GetVisualizeData();

  rclcpp::QoS qos(10);

  subLeftImg_ = create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera_gray_left/image_raw", qos,
    std::bind(&SlamNode::leftCameraHandler, this, std::placeholders::_1));

  subRightImg_ = create_subscription<sensor_msgs::msg::Image>(
    "kitti/camera_gray_right/image_raw", qos,
    std::bind(&SlamNode::rightCameraHandler, this, std::placeholders::_1));

  // sync gps and imu msg
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  subGPS_.subscribe(this, "kitti/oxts/gps/fix", rmw_qos_profile);
  subIMU_.subscribe(this, "kitti/oxts/imu", rmw_qos_profile);
  sync_.registerCallback(&SlamNode::sync_callback, this);

  pubImg_ = create_publisher<sensor_msgs::msg::Image>("stereo_slam/annotated_img", qos);
  pubCameraPose_ = create_publisher<nav_msgs::msg::Odometry>("stereo_slam/estimated_pose", qos);
  pubMapPoint_ = create_publisher<sensor_msgs::msg::PointCloud>("stereo_slam/map_points", qos);
  pubGpsPath_ = create_publisher<nav_msgs::msg::Path>("stereo_slam/gps_path", qos);
  pubCameraPath_ = create_publisher<nav_msgs::msg::Path>("stereo_slam/camera_path", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void SlamNode::sync_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imuMsg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gpsMsg)
{
  // use the first gps signal as (0,0,0)
  if (!gps_imu_init_) {
    geo_converter_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    Eigen::Quaterniond init_rot;
    tf2::fromMsg(imuMsg->orientation, init_rot);
    initial_rotation_ = init_rot.toRotationMatrix().transpose();
    gps_imu_init_ = true;
  }

  // orientation information from imu_msg
  Eigen::Quaterniond imu_orientation(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z);

  // pose information from gps_msg
  double lat, lon, alt;
  geo_converter_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, lat, lon, alt);

  // rotate the pose with init_rotation_
  Eigen::Vector3d pose = initial_rotation_ * Eigen::Vector3d(lat, lon, alt);

  // rotate orientation and convert to quaternion
  Eigen::Quaterniond orientation(imu_orientation.toRotationMatrix() * initial_rotation_);

  // publish gps path
  geometry_msgs::msg::PoseStamped gpsPose;
  gpsPose.header.frame_id = "odom";
  gpsPose.header.stamp = rclcpp::Node::now();
  gpsPose.pose.position.x = pose.x();
  gpsPose.pose.position.y = pose.y();
  gpsPose.pose.position.z = pose.z();
  gpsPose.pose.orientation.x = orientation.x();
  gpsPose.pose.orientation.y = orientation.y();
  gpsPose.pose.orientation.z = orientation.z();
  gpsPose.pose.orientation.w = orientation.w();
  gpsPath_.header = gpsPose.header;
  gpsPath_.poses.push_back(gpsPose);
  pubGpsPath_->publish(gpsPath_);

  // publish tf msg
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Node::now();
  t.header.frame_id = "odom";
  t.child_frame_id = "gps_link";
  t.transform.translation.x = pose.x();
  t.transform.translation.y = pose.y();
  t.transform.translation.z = pose.z();
  t.transform.rotation.x = orientation.x();
  t.transform.rotation.y = orientation.y();
  t.transform.rotation.z = orientation.z();
  t.transform.rotation.w = orientation.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
};

void SlamNode::image_processing()
{
  // TODO: change to false when slam is done.
  while (!pvo_->exit_) {
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

void SlamNode::pubVisualizeData()
{
  while (!pvo_->exit_) {
    // pub image
    if (!viewer_->img_out_buf_.empty()) {
      std::lock_guard<std::mutex> lck(img_mutex_);
      cv::Mat img = viewer_->img_out_buf_.front();
      viewer_->img_out_buf_.pop();
      sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      img_msg->header.stamp = rclcpp::Node::now();
      pubImg_->publish(*img_msg);
    }

    // pub frame pose & tf2_msg
    if (!viewer_->pose_out_buf_.empty()) {
      std::lock_guard<std::mutex> lck(img_mutex_);
      Sophus::SE3d se3_pose = viewer_->pose_out_buf_.front();
      viewer_->pose_out_buf_.pop();
      // change coordinate to align "odom frame"
      se3_pose = Sophus::SE3d::rotY(M_PI/2.0) * Sophus::SE3d::rotZ(-M_PI/2.0) * se3_pose;
      pubTF2(se3_pose);
      nav_msgs::msg::Odometry odom_msg = sophusToMsg(se3_pose);
      pubCameraPose_->publish(odom_msg);

      // publish path
      geometry_msgs::msg::PoseStamped cameraPose;
      cameraPose.header = odom_msg.header;
      cameraPose.pose = odom_msg.pose.pose;
      cameraPath_.header = odom_msg.header;
      cameraPath_.poses.push_back(cameraPose);
      pubCameraPath_->publish(cameraPath_);
    }

    // pub map point
    if (!viewer_->mappoint_out_buf_.empty()) {
      std::lock_guard<std::mutex> lck(img_mutex_);
      std::vector<Vec3> mp = viewer_->mappoint_out_buf_.front();
      viewer_->mappoint_out_buf_.pop();
      sensor_msgs::msg::PointCloud mp_msg = eigenToMsg(mp);
      pubMapPoint_->publish(mp_msg);
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

nav_msgs::msg::Odometry SlamNode::sophusToMsg(const Sophus::SE3d& se3)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "camera_link";
  msg.header.stamp = rclcpp::Node::now();
  msg.pose.pose.position.x = se3.translation().x();
  msg.pose.pose.position.y = se3.translation().y();
  msg.pose.pose.position.z = se3.translation().z();
  msg.pose.pose.orientation.x = se3.unit_quaternion().x();
  msg.pose.pose.orientation.y = se3.unit_quaternion().y();
  msg.pose.pose.orientation.z = se3.unit_quaternion().z();
  msg.pose.pose.orientation.w = se3.unit_quaternion().w();
  return msg;
}

sensor_msgs::msg::PointCloud SlamNode::eigenToMsg(const std::vector<Vec3>& points)
{
  // Create rotation matrix to tranform point clouds to odom frame
  Eigen::AngleAxisd pitchAngle(M_PI/2.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(-M_PI/2.0, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q =  pitchAngle * yawAngle;
  Eigen::Matrix3d rotationMatrix = q.matrix();

  sensor_msgs::msg::PointCloud map;
  map.header.frame_id = "odom";
  map.header.stamp = rclcpp::Node::now();
  for (const auto& point: points) {
    Vec3 point_world = rotationMatrix * point;
    geometry_msgs::msg::Point32 p;
    p.x = point_world.x();
    p.y = point_world.y();
    p.z = point_world.z();
    map.points.push_back(p);
  }
  return map;
}

void SlamNode::pubTF2(const Sophus::SE3d& se3)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = rclcpp::Node::now();
  t.header.frame_id = "odom";
  t.child_frame_id = "camera_link";
  t.transform.translation.x = se3.translation().x();
  t.transform.translation.y = se3.translation().y();
  t.transform.translation.z = se3.translation().z();
  t.transform.rotation.x = se3.unit_quaternion().x();
  t.transform.rotation.y = se3.unit_quaternion().y();
  t.transform.rotation.z = se3.unit_quaternion().z();
  t.transform.rotation.w = se3.unit_quaternion().w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}
