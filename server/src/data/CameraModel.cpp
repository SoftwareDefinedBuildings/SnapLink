#include "CameraModel.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <cassert>
#include <opencv2/imgproc/imgproc.hpp>

CameraModel::CameraModel() = default;

CameraModel::CameraModel(
    const std::string &name,
    double fx,
    double fy,
    double cx,
    double cy,
    Transform &&localTransform,
    double Tx,
    const cv::Size &imageSize) :
    name_(name),
    imageSize_(imageSize),
    K_(cv::Mat::eye(3, 3, CV_64FC1)),
    localTransform_(std::move(localTransform))
{
    assert(fx > 0.0);
    assert(fy > 0.0);
    assert(cx >= 0.0);
    assert(cy >= 0.0);
    assert(!localTransform.isNull());
    if (Tx != 0.0)
    {
        P_ = cv::Mat::eye(3, 4, CV_64FC1),
        P_.at<double>(0, 0) = fx;
        P_.at<double>(1, 1) = fy;
        P_.at<double>(0, 2) = cx;
        P_.at<double>(1, 2) = cy;
        P_.at<double>(0, 3) = Tx;
    }

    K_.at<double>(0, 0) = fx;
    K_.at<double>(1, 1) = fy;
    K_.at<double>(0, 2) = cx;
    K_.at<double>(1, 2) = cy;
}

CameraModel::~CameraModel() = default;
