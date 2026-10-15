#pragma once

#include <cmath>
#include <opencv2/core.hpp>

struct AimError {
  float yaw;
  float pitch;

  float pixel_x;
  float pixel_y;
};

class AimController {

public:
  AimController(float fx, float fy, float cx, float cy);

  AimError calculate(const cv::Point2f &target);

private:
  float fx_;
  float fy_;

  float cx_;
  float cy_;
};