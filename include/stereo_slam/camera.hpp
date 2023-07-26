#pragma once

#include <sophus/se3.hpp>

// local header
#include "stereo_slam/camera.hpp"
#include "stereo_slam/common.hpp"

class Camera
{
  public:
    using Ptr = std::shared_ptr<Camera>;

    Camera() = default;
    Camera(double fx, double fy, double cx, double cy, double baseline,
           const Sophus::SE3d& pose);

    Sophus::SE3d pose() const;

    Mat33 K() const;

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3& p_w, const Sophus::SE3d& T_c_w);

    Vec3 camera2world(const Vec3& p_c, const Sophus::SE3d& T_c_w);

    Vec2 camera2pixel(const Vec3& p_c);

    Vec3 pixel2camera(const Vec2& p_p, double depth = 1);

    Vec2 world2pixel(const Vec3& p_w, const Sophus::SE3d& T_c_w);

    Vec3 pixel2world(const Vec2& p_p, const Sophus::SE3d& T_c_w, double depth = 1);

  private:
    // Camera intrinsics
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double baseline_;

    // extrinsic, from stereo camera to single camera
    Sophus::SE3d pose_;

    // inverse of extrinsics
    Sophus::SE3d pose_inv_;
};
