#include "CameraModel.h"
#include <cassert>
#include <opencv2/imgproc/imgproc.hpp>

CameraModel::CameraModel() = default;

CameraModel::CameraModel(const std::string &name, double fx, double fy,
                         double cx, double cy, cv::Size &&imageSize)
    : _name(name), _imageSize(imageSize), _K(cv::Mat::eye(3, 3, CV_64FC1)) {
  assert(fx > 0.0);
  assert(fy > 0.0);
  assert(cx >= 0.0);
  assert(cy >= 0.0);

  _K.at<double>(0, 0) = fx;
  _K.at<double>(1, 1) = fy;
  _K.at<double>(0, 2) = cx;
  _K.at<double>(1, 2) = cy;

  assert(!_K.empty());
}

CameraModel::~CameraModel() = default;

const std::string &CameraModel::name() const { return _name; }

double CameraModel::fx() const { return _K.at<double>(0, 0); }

double CameraModel::fy() const { return _K.at<double>(1, 1); }

double CameraModel::cx() const { return _K.at<double>(0, 2); }

double CameraModel::cy() const { return _K.at<double>(1, 2); }

cv::Mat CameraModel::K() const { return _K; }

cv::Mat CameraModel::D() const {
  return !_D.empty() ? _D : cv::Mat::zeros(1, 5, CV_64FC1);
}

const cv::Size &CameraModel::getImageSize() const { return _imageSize; }
