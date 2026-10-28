#include "ArmorPoseSolver.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

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

  for (const cv::Point2f &p : imagePoints) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return false;
    }
  }

  // 2. 定义装甲板在自身坐标系下的四个三维角点。
  //
  // 这里沿用现有项目中的尺寸：
  // halfWidth = 0.03125 m
  // halfBarLength = 0.01425 m
  //
  // 对应模型宽度 0.0625 m、高度 0.0285 m。
  //
  // 该坐标顺序必须与 imagePoints 的角点顺序严格对应：
  // 左上、右上、右下、左下。
  constexpr double halfWidth = 0.03125;
  constexpr double halfHeight = 0.01425;

  const std::vector<cv::Point3f> objectPoints = {
      cv::Point3f(-halfWidth, -halfHeight, 0.0f), // 左上
      cv::Point3f(halfWidth, -halfHeight, 0.0f),  // 右上
      cv::Point3f(halfWidth, halfHeight, 0.0f),   // 右下
      cv::Point3f(-halfWidth, halfHeight, 0.0f)   // 左下
  };

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

  // 4. 平移向量必须是有限数值，且目标必须位于相机前方。
  if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) ||
      !std::isfinite(tvec[2]) || tvec[2] <= 0.0) {
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
    const double error = cv::norm(points[i] - projectedPoints[i]);

    if (!std::isfinite(error)) {
      return false;
    }

    totalError += error;
  }

  const double meanError = totalError / static_cast<double>(points.size());

  // 6. 保存求解结果。
  pose.translation = tvec;
  pose.rotation = rvec;

  // 三维欧氏距离，不只是 Z 方向距离。
  pose.distance = cv::norm(tvec);

  pose.reprojectionError = meanError;
  pose.valid = true;

  return true;
}
