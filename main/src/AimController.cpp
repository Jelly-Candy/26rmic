#include "AimController.hpp"

AimController::AimController(float fx, float fy, float cx, float cy)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

AimError AimController::calculate(const cv::Point2f &target) {

  AimError result;

  float dx = target.x - cx_;

  float dy = target.y - cy_;

  result.pixel_x = dx;
  result.pixel_y = dy;

  result.yaw = std::atan(dx / fx_) * 180.0f / CV_PI;

  result.pitch = std::atan(dy / fy_) * 180.0f / CV_PI;

  return result;
}