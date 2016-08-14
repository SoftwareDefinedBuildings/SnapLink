#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "data/CameraModel.h"

class SensorData
{
public:
    SensorData();

    SensorData(
        const cv::Mat &image,
        const cv::Mat &depth,
        CameraModel &&cameraModel,
        int id = 0,
        double stamp = 0.0);

    virtual ~SensorData();

    int getId() const;
    void setId(int id);
    double getStamp() const;
    const cv::Mat &getImage() const;
    const cv::Mat &getDepth() const;
    const CameraModel &getCameraModel() const;

    void setFeatures(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descriptors)
    {
        _keypoints = keypoints;
        _descriptors = descriptors;
    }
    const std::vector<cv::KeyPoint> &keypoints() const
    {
        return _keypoints;
    }
    const cv::Mat &descriptors() const
    {
        return _descriptors;
    }

private:
    int _id;
    double _stamp;

    cv::Mat _image;          // CV_8UC1 or CV_8UC3
    cv::Mat _depth;         // depth CV_16UC1 or CV_32FC1, right image CV_8UC1

    CameraModel _cameraModel;

    // features
    std::vector<cv::KeyPoint> _keypoints;
    cv::Mat _descriptors;
};
