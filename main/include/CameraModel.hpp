#pragma once

#include <opencv2/core.hpp>
#include <string>


class CameraModel
{
public:

    CameraModel();

    bool load(const std::string& filename);


    // 像素坐标转换为角度
    cv::Point2f pixelToAngle(
        const cv::Point2f& pixel
    ) const;

bool setImageSize(int width, int height);

private:
    float baseFx_, baseFy_, baseCx_, baseCy_;
    float fx_, fy_, cx_, cy_;

    int calibrationWidth_;
    int calibrationHeight_;
    int imageWidth_;
    int imageHeight_;

    bool loaded_;
};