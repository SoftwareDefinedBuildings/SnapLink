#pragma once

#include "data/CameraModel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class SensorData {
public:
  SensorData(cv::Mat &&image, CameraModel &&cameraModel);

  const cv::Mat &getImage() const;
  const CameraModel &getCameraModel() const;

  void setFeatures(const std::vector<cv::KeyPoint> &keypoints,
                   const cv::Mat &descriptors);
  const std::vector<cv::KeyPoint> &keypoints() const;
  const cv::Mat &descriptors() const;

private:
  cv::Mat _image; // CV_8UC1 or CV_8UC3

  CameraModel _cameraModel;

  // features
  std::vector<cv::KeyPoint> _keypoints;
  cv::Mat _descriptors;
};
