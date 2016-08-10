#pragma once

#include "data/Transform.h"
#include <opencv2/opencv.hpp>

class CameraModel
{
public:
    CameraModel();

    // minimal to be saved
    CameraModel(
        const std::string &name,
        double fx,
        double fy,
        double cx,
        double cy,
        Transform &&localTransform,
        double Tx = 0.0f,
        const cv::Size &imageSize = cv::Size(0, 0));

    virtual ~CameraModel();

    bool isValidForProjection() const
    {
        return fx() > 0.0 && fy() > 0.0;
    }

    const std::string &name() const
    {
        return name_;
    }

    double fx() const
    {
        return P_.empty() ? K_.empty() ? 0.0 : K_.at<double>(0, 0) : P_.at<double>(0, 0);
    }
    double fy() const
    {
        return P_.empty() ? K_.empty() ? 0.0 : K_.at<double>(1, 1) : P_.at<double>(1, 1);
    }
    double cx() const
    {
        return P_.empty() ? K_.empty() ? 0.0 : K_.at<double>(0, 2) : P_.at<double>(0, 2);
    }
    double cy() const
    {
        return P_.empty() ? K_.empty() ? 0.0 : K_.at<double>(1, 2) : P_.at<double>(1, 2);
    }
    double Tx() const
    {
        return P_.empty() ? 0.0 : P_.at<double>(0, 3);
    }

    cv::Mat K() const
    {
        return !P_.empty() ? P_.colRange(0, 3) : K_;   // if P exists, return rectified version
    }
    cv::Mat D() const
    {
        return P_.empty() && !D_.empty() ? D_ : cv::Mat::zeros(1, 5, CV_64FC1);   // if P exists, return rectified version
    }
    cv::Mat P() const
    {
        return P_;   //projection matrix
    }

    void setLocalTransform(const Transform &transform)
    {
        localTransform_ = transform;
    }
    const Transform &localTransform() const
    {
        return localTransform_;
    }

    void setImageSize(const cv::Size &size)
    {
        imageSize_ = size;
    }
    const cv::Size &imageSize() const
    {
        return imageSize_;
    }
    int imageWidth() const
    {
        return imageSize_.width;
    }
    int imageHeight() const
    {
        return imageSize_.height;
    }

private:
    std::string name_;
    cv::Size imageSize_;
    cv::Mat K_;
    cv::Mat D_;
    cv::Mat P_;
    cv::Mat mapX_;
    cv::Mat mapY_;
    Transform localTransform_;
};
