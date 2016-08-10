#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class SensorData
{
public:
    SensorData();

    SensorData(
            const cv::Mat & image,
            const CameraModel & cameraModel,
            int id = 0,
            double stamp = 0.0,
            const cv::Mat & userData = cv::Mat());

    virtual ~SensorData();

    int id() const {return _id;}
    void setId(int id) {_id = id;}
    double stamp() const {return _stamp;}
    void setStamp(double stamp) {_stamp = stamp;}
    int laserScanMaxPts() const {return _laserScanMaxPts;}
    float laserScanMaxRange() const {return _laserScanMaxRange;}

    const cv::Mat & imageRaw() const {return _imageRaw;}

    void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors)
    {
        _keypoints = keypoints;
        _descriptors = descriptors;
    }
    const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
    const cv::Mat & descriptors() const {return _descriptors;}

private:
    int _id;
    double _stamp;

    cv::Mat _imageRaw;          // CV_8UC1 or CV_8UC3

    // features
    std::vector<cv::KeyPoint> _keypoints;
    cv::Mat _descriptors;
};
