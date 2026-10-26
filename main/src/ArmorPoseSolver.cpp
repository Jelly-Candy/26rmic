#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <array>

struct ArmorPose
{
    bool valid = false;
    cv::Vec3d translation{};  // 相机坐标系下的平移，单位 m
    cv::Vec3d rotation{};     // Rodrigues 旋转向量
    double distance = 0.0;    // 相机到目标原点的直线距离，单位 m
};

bool solveSmallArmorPnP(
    const std::array<cv::Point2f, 4>& imagePoints,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    ArmorPose& pose)
{
    constexpr double halfWidth = 0.03125;
    constexpr double halfBarLength = 0.01425;

    const std::vector<cv::Point3f> objectPoints = {
        {-halfWidth, -halfBarLength, 0.0f},
        { halfWidth, -halfBarLength, 0.0f},
        { halfWidth,  halfBarLength, 0.0f},
        {-halfWidth,  halfBarLength, 0.0f}
    };

    cv::Vec3d rvec, tvec;

    const bool ok = cv::solvePnP(
        objectPoints,
        std::vector<cv::Point2f>(
            imagePoints.begin(), imagePoints.end()),
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );

    if (!ok || tvec[2] <= 0.0)
        return false;

    pose.rotation = rvec;
    pose.translation = tvec;
    pose.distance = cv::norm(tvec);
    pose.valid = true;
    return true;
}