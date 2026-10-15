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


private:

    float fx_;
    float fy_;
    float cx_;
    float cy_;

    bool loaded_;
};