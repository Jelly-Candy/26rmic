#pragma once

#include <opencv2/core.hpp>
#include <string>

class CameraModel {
public:
  CameraModel();

  bool load(const std::string &filename);

  // 像素坐标转换为角度
  cv::Point2f pixelToAngle(const cv::Point2f &pixel) const;

  bool setImageSize(int width, int height);
  cv::Point2f mountedPixelToAngle(const cv::Point2f &pixel) const;
  cv::Point2f mountedPrincipalPoint() const;
  // 返回供 PnP 使用的相机内参和畸变参数。
  bool getPnPParameters(cv::Mat &cameraMatrix, cv::Mat &distCoeffs) const;

private:
  float baseFx_, baseFy_, baseCx_, baseCy_;
  float fx_, fy_, cx_, cy_;

  int calibrationWidth_;
  int calibrationHeight_;
  int imageWidth_;
  int imageHeight_;

  bool loaded_;

  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
};