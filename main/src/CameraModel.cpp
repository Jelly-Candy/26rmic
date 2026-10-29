#include "CameraModel.hpp"

#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>

CameraModel::CameraModel() : fx_(0), fy_(0), cx_(0), cy_(0), loaded_(false) {}

bool CameraModel::load(const std::string &filename) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::printf("Failed to open calibration file: %s\n", filename.c_str());
    return false;
  }

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

  if (cameraMatrix.empty() || cameraMatrix.rows != 3 ||
      cameraMatrix.cols != 3) {
    std::printf("Invalid camera_matrix\n");
    return false;
  }

  if (distCoeffs.empty()) {
    std::printf("Missing distortion_coefficients\n");
    return false;
  }

  cameraMatrix.convertTo(cameraMatrix_, CV_64F);
  distCoeffs.convertTo(distCoeffs_, CV_64F);

  if (cameraMatrix.empty() || cameraMatrix.rows != 3 ||
      cameraMatrix.cols != 3) {
    std::printf("Invalid camera_matrix\n");
    return false;
  }

  // 读取标定时的图像分辨率
  fs["image_width"] >> calibrationWidth_;
  fs["image_height"] >> calibrationHeight_;

  if (calibrationWidth_ <= 0 || calibrationHeight_ <= 0) {
    std::printf("Invalid calibration image size: %d x %d\n", calibrationWidth_,
                calibrationHeight_);
    return false;
  }

  // 转换后的内参必须是有限数值，焦距必须为正。
  if (!cv::checkRange(cameraMatrix_) || !cv::checkRange(distCoeffs_)) {
    std::printf("Invalid camera calibration: non-finite values\n");
    return false;
  }

  if (cameraMatrix_.at<double>(0, 0) <= 0.0 ||
      cameraMatrix_.at<double>(1, 1) <= 0.0) {
    std::printf("Invalid camera calibration: focal length must be positive\n");
    return false;
  }
  baseFx_ = static_cast<float>(cameraMatrix_.at<double>(0, 0));
  baseFy_ = static_cast<float>(cameraMatrix_.at<double>(1, 1));
  baseCx_ = static_cast<float>(cameraMatrix_.at<double>(0, 2));
  baseCy_ = static_cast<float>(cameraMatrix_.at<double>(1, 2));

  // 初始有效内参等于原始内参
  fx_ = baseFx_;
  fy_ = baseFy_;
  cx_ = baseCx_;
  cy_ = baseCy_;

  loaded_ = true;

  std::printf("Calibration loaded: image=%dx%d, fx=%.2f, fy=%.2f, "
              "cx=%.2f, cy=%.2f\n",
              calibrationWidth_, calibrationHeight_, fx_, fy_, cx_, cy_);

  return true;
}

cv::Point2f CameraModel::pixelToAngle(const cv::Point2f &pixel) const {

  std::printf("pixel=(%.1f, %.1f), cx=%.2f, cy=%.2f, fx=%.2f, fy=%.2f\n",
              pixel.x, pixel.y, cx_, cy_, fx_, fy_);
  if (!loaded_) {
    return cv::Point2f(0, 0);
  }

  float yaw = std::atan((pixel.x - cx_) / fx_) * 180.0f / CV_PI;

  float pitch = std::atan((cy_ - pixel.y) / fy_) * 180.0f / CV_PI;

  return cv::Point2f(yaw, pitch);
}

bool CameraModel::setImageSize(int width, int height) {
  if (!loaded_ || width <= 0 || height <= 0) {
    return false;
  }

  // 新标定文件必须与实际图像分辨率一致
  if (width != calibrationWidth_ || height != calibrationHeight_) {
    std::printf("Image size mismatch: calibration=%dx%d, runtime=%dx%d\n",
                calibrationWidth_, calibrationHeight_, width, height);
    return false;
  }

  // 直接使用标定内参，不进行缩放
  fx_ = baseFx_;
  fy_ = baseFy_;
  cx_ = baseCx_;
  cy_ = baseCy_;

  imageWidth_ = width;
  imageHeight_ = height;

  std::printf("Using native calibration: %dx%d, "
              "fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f\n",
              width, height, fx_, fy_, cx_, cy_);

  return true;
}
// The raw calibration belongs to the unrotated sensor image. A 180-degree
// roll negates both optical ray components, but not the positive focal lengths.
cv::Point2f CameraModel::mountedPixelToAngle(const cv::Point2f &pixel) const {
  const cv::Point2f raw(float(imageWidth_ - 1) - pixel.x,
                        float(imageHeight_ - 1) - pixel.y);
  const cv::Point2f rawAngle = pixelToAngle(raw);
  return cv::Point2f(-rawAngle.x, -rawAngle.y);
}
cv::Point2f CameraModel::mountedPrincipalPoint() const {
  return cv::Point2f(float(imageWidth_ - 1) - cx_,
                     float(imageHeight_ - 1) - cy_);
}

bool CameraModel::getPnPParameters(cv::Mat &cameraMatrix,
                                   cv::Mat &distCoeffs) const {
  if (!loaded_ || cameraMatrix_.empty() || distCoeffs_.empty()) {
    return false;
  }

  cameraMatrix = cameraMatrix_.clone();
  distCoeffs = distCoeffs_.clone();
  return true;
}