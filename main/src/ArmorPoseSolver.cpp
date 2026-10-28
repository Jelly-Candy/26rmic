
#include "ArmorPoseSolver.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

// 求解小装甲板的 PnP 姿态。
bool solveSmallArmorPnP(const std::array<cv::Point2f, 4> &imagePoints,
                        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                        ArmorPose &pose) {
  // 每次求解前清空上一次结果，避免失败时残留旧数据。
  pose = ArmorPose{};

  // 1. 检查输入参数。
  if (cameraMatrix.empty() || cameraMatrix.rows != 3 ||
      cameraMatrix.cols != 3 || distCoeffs.empty()) {
    return false;
  }

  if (cameraMatrix.type() != CV_64F ||
      (distCoeffs.depth() != CV_64F && distCoeffs.depth() != CV_32F)) {
    return false;
  }

  for (const cv::Point2f &p : imagePoints) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return false;
    }
  }

  // 2. 定义装甲板在自身坐标系下的四个三维角点。
  // 物点顺序必须与 imagePoints 的顺序一致：
  // 左上、右上、右下、左下。
  constexpr double halfWidth = 0.03125;
  constexpr double halfHeight = 0.01425;

  const std::vector<cv::Point3f> objectPoints = {
      cv::Point3f(-halfWidth, -halfHeight, 0.0f),
      cv::Point3f(halfWidth, -halfHeight, 0.0f),
      cv::Point3f(halfWidth, halfHeight, 0.0f),
      cv::Point3f(-halfWidth, halfHeight, 0.0f)};

  const std::vector<cv::Point2f> points(imagePoints.begin(), imagePoints.end());

  // 3. 使用 PnP 求解旋转向量和平移向量。
  cv::Vec3d rvec;
  cv::Vec3d tvec;

  const bool solved =
      cv::solvePnP(objectPoints, points, cameraMatrix, distCoeffs, rvec, tvec,
                   false, cv::SOLVEPNP_ITERATIVE);

  if (!solved) {
    return false;
  }

  // 4. 旋转向量和平移向量都必须是有限数值。
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
      return false;
    }
  }

  // 目标必须位于相机前方。
  if (tvec[2] <= 0.0) {
    return false;
  }

  // 5. 将三维模型角点重新投影到图像，计算平均误差。
  std::vector<cv::Point2f> projectedPoints;

  cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs,
                    projectedPoints);

  if (projectedPoints.size() != points.size()) {
    return false;
  }

  double totalError = 0.0;

  for (std::size_t i = 0; i < points.size(); ++i) {
    const cv::Point2f &p = projectedPoints[i];

    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return false;
    }

    const double error = cv::norm(points[i] - p);

    if (!std::isfinite(error)) {
      return false;
    }

    totalError += error;
  }

  const double meanError = totalError / static_cast<double>(points.size());

  const double distance = cv::norm(tvec);

  if (!std::isfinite(meanError) || !std::isfinite(distance) ||
      distance <= 0.0) {
    return false;
  }

  // 6. 保存求解结果。
  pose.translation = tvec;
  pose.rotation = rvec;
  pose.distance = distance;
  pose.reprojectionError = meanError;
  pose.valid = true;

  return true;
}
