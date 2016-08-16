#pragma once

#include "data/Transform.h"
#include <opencv2/opencv.hpp>

class CameraModel {
public:
  CameraModel();

  // minimal to be saved
  CameraModel(const std::string &name, double fx, double fy, double cx,
              double cy, cv::Size &&imageSize);

  virtual ~CameraModel();

  const std::string &name() const;
  double fx() const;
  double fy() const;
  double cx() const;
  double cy() const;

  cv::Mat K() const;
  cv::Mat D() const;
  cv::Mat P() const;

  const cv::Size &getImageSize() const;

private:
  std::string _name;
  cv::Size _imageSize;
  cv::Mat _K;
  cv::Mat _D;
};
