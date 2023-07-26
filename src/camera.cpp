#include "stereo_slam/camera.hpp"


Camera::Camera(double fx, double fy, double cx, double cy, double baseline,
               const Sophus::SE3d& pose)
  : fx_{fx}, fy_{fy}, cx_{cx}, cy_{cy}, baseline_{baseline}, pose_{pose},
    pose_inv_{pose.inverse()}
{
}

Sophus::SE3d Camera::pose() const
{
  return pose_;
}

Mat33 Camera::K() const
{
  Mat33 k;
  k << fx_, 0.0, cx_,
       0.0, fy_, cy_,
       0.0, 0.0, 1.0;
  return k;
}

Vec3 Camera::world2camera(const Vec3& p_w, const Sophus::SE3d& T_c_w)
{
  return pose_ * T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3& p_c, const Sophus::SE3d& T_c_w)
{
  return T_c_w.inverse() * pose_inv_ * p_c;
}

Vec2 Camera::camera2pixel(const Vec3& p_c)
{
  return Vec2(
    fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
    fy_ * p_c(1, 0) / p_c(2, 0) + cy_
  );
}

Vec3 Camera::pixel2camera(const Vec2& p_p, double depth)
{
  return Vec3(
    (p_p(0, 0) - cx_) * depth / fx_,
    (p_p(1, 0) - cy_) * depth / fy_,
    depth
  );
}

Vec2 Camera::world2pixel(const Vec3& p_w, const Sophus::SE3d& T_c_w)
{
  return camera2pixel(world2camera(p_w, T_c_w));
}

Vec3 Camera::pixel2world(const Vec2& p_p, const Sophus::SE3d& T_c_w, double depth)
{
  return camera2world(pixel2camera(p_p, depth), T_c_w);
}
