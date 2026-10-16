#include "CameraModel.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>

CameraModel::CameraModel() : fx_(0), fy_(0), cx_(0), cy_(0), loaded_(false) {}

bool CameraModel::load(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::printf("Failed to open calibration file: %s\n",
                    filename.c_str());
        return false;
    }

    cv::Mat cameraMatrix;
    fs["camera_matrix"] >> cameraMatrix;

    if (cameraMatrix.empty() ||
        cameraMatrix.rows != 3 ||
        cameraMatrix.cols != 3)
    {
        std::printf("Invalid camera_matrix\n");
        return false;
    }

    // 读取标定时的图像分辨率
    fs["image_width"] >> calibrationWidth_;
    fs["image_height"] >> calibrationHeight_;

    if (calibrationWidth_ <= 0 || calibrationHeight_ <= 0)
    {
        std::printf("Invalid calibration image size: %d x %d\n",
                    calibrationWidth_, calibrationHeight_);
        return false;
    }

    // 保存原始内参
    baseFx_ = static_cast<float>(cameraMatrix.at<double>(0, 0));
    baseFy_ = static_cast<float>(cameraMatrix.at<double>(1, 1));
    baseCx_ = static_cast<float>(cameraMatrix.at<double>(0, 2));
    baseCy_ = static_cast<float>(cameraMatrix.at<double>(1, 2));

    // 初始有效内参等于原始内参
    fx_ = baseFx_;
    fy_ = baseFy_;
    cx_ = baseCx_;
    cy_ = baseCy_;

    loaded_ = true;

    std::printf(
        "Calibration loaded: image=%dx%d, fx=%.2f, fy=%.2f, "
        "cx=%.2f, cy=%.2f\n",
        calibrationWidth_, calibrationHeight_,
        fx_, fy_, cx_, cy_);

    return true;
}

cv::Point2f CameraModel::pixelToAngle(const cv::Point2f &pixel) const {

  std::printf("pixel=(%.1f, %.1f), cx=%.2f, cy=%.2f, fx=%.2f, fy=%.2f\n",
              pixel.x, pixel.y, cx_, cy_, fx_, fy_);
  if (!loaded_) {
    return cv::Point2f(0, 0);
  }

  float x = (pixel.x - cx_) / fx_;

  float y = (pixel.y - cy_) / fy_;

  float yaw = std::atan((pixel.x - cx_) / fx_) * 180.0f / CV_PI;

  float pitch = std::atan((cy_ - pixel.y) / fy_) * 180.0f / CV_PI;

  return cv::Point2f(yaw, pitch);
}

bool CameraModel::setImageSize(int width, int height) {
  if (!loaded_ || width <= 0 || height <= 0 || calibrationWidth_ <= 0 ||
      calibrationHeight_ <= 0) {
    return false;
  }

  const float sx = static_cast<float>(width) / calibrationWidth_;
  const float sy = static_cast<float>(height) / calibrationHeight_;

  fx_ = baseFx_ * sx;
  fy_ = baseFy_ * sy;
  cx_ = baseCx_ * sx;
  cy_ = baseCy_ * sy;

  imageWidth_ = width;
  imageHeight_ = height;

  return true;
}