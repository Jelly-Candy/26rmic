#pragma once

#include <array>
#include <opencv2/core.hpp>

struct ArmorPose {
    bool valid = false;
    cv::Vec3d translation{};  // 相机坐标系，单位 m
    cv::Vec3d rotation{};     // Rodrigues 旋转向量
    double distance = 0.0;    // 相机到目标中心的直线距离，单位 m
    double reprojectionError = 0.0;   // 四个角点的平均重投影误差，单位：像素
};

bool solveSmallArmorPnP(
    const std::array<cv::Point2f, 4> &imagePoints,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    ArmorPose &pose);