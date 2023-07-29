#pragma once

// c++ header
#include <memory>

// opencv
#include <Eigen/Core>

// define types
using Mat22 = Eigen::Matrix<double, 2, 2>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Mat34 = Eigen::Matrix<double, 3, 4>;
using MatXX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;
