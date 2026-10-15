#include "CameraModel.hpp"

#include <opencv2/core.hpp>

#include <cmath>


CameraModel::CameraModel()
    :
    fx_(0),
    fy_(0),
    cx_(0),
    cy_(0),
    loaded_(false)
{

}



bool CameraModel::load(
    const std::string& filename
)
{
    cv::FileStorage fs(
        filename,
        cv::FileStorage::READ
    );


    if(!fs.isOpened())
    {
        return false;
    }


    cv::Mat cameraMatrix;


    fs["camera_matrix"] >> cameraMatrix;


    if(cameraMatrix.empty())
    {
        return false;
    }


    fx_ = cameraMatrix.at<double>(0,0);
    fy_ = cameraMatrix.at<double>(1,1);

    cx_ = cameraMatrix.at<double>(0,2);
    cy_ = cameraMatrix.at<double>(1,2);


    loaded_ = true;


    return true;
}



cv::Point2f CameraModel::pixelToAngle(
    const cv::Point2f& pixel
) const
{

    if(!loaded_)
    {
        return cv::Point2f(0,0);
    }


    float x =
        (pixel.x - cx_) / fx_;


    float y =
        (pixel.y - cy_) / fy_;


    float yaw =
        std::atan(x)
        *
        180.0f
        /
        CV_PI;


    float pitch =
        std::atan(y)
        *
        180.0f
        /
        CV_PI;



    return cv::Point2f(
        yaw,
        pitch
    );
}