#pragma once

#include <opencv2/opencv.hpp>

class CameraModel final {
public:
  explicit CameraModel();

  explicit CameraModel(std::string &&name, double fx, double fy, double cx, double cy,
              cv::Size &&imageSize);

  const std::string &name() const;
  double fx() const;
  double fy() const;
  double cx() const;
  double cy() const;

  const cv::Mat &K() const;
  const cv::Mat &D() const;

  const cv::Size &getImageSize() const;

private:
  std::string _name;
  cv::Mat _K;
  cv::Mat _D;
  cv::Size _imageSize;
};
