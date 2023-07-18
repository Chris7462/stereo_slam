#include <iostream>
#include <chrono>
#include <thread>

#include "stereo_slam/visual_odometry.hpp"

VisualOdometry::VisualOdometry()
{
}

void VisualOdometry::Init()
{
  std::cout << "In the visual_odometry init" << std::endl;
}

void VisualOdometry::Run()
{
  while (1) {
    std::cout << "In the visual_odometry run" << std::endl;

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}
