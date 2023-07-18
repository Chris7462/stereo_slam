#pragma once

// c++ hader
#include <memory>

// local header
#include "stereo_slam/frontend.hpp"

class VisualOdometry
{
  public:
    VisualOdometry();
    void Init();
    void Run();
  //void Shutdown();

  private:
    std::shared_ptr<Frontend> frontend_;
};
