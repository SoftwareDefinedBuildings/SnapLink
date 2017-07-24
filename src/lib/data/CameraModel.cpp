#include "lib/data/CameraModel.h"
#include <cassert>
#include <opencv2/core/core.hpp>

CameraModel::CameraModel() = default;

CameraModel::CameraModel(const std::string &name, double fx, double fy,
                         double cx, double cy, const cv::Size &imageSize)
    : _name(name), _K(cv::Mat::eye(3, 3, CV_64FC1)),
      _D(cv::Mat::zeros(1, 5, CV_64FC1)), _imageSize(imageSize) {

  _K.at<double>(0, 0) = fx;
  _K.at<double>(1, 1) = fy;
  _K.at<double>(0, 2) = cx;
  _K.at<double>(1, 2) = cy;

  assert(!_K.empty());
  assert(_K.rows == 3 && _K.cols == 3);
  assert(_D.rows == 1 && _D.cols == 5);
}

const std::string &CameraModel::name() const { return _name; }

double CameraModel::fx() const { return _K.at<double>(0, 0); }

double CameraModel::fy() const { return _K.at<double>(1, 1); }

double CameraModel::cx() const { return _K.at<double>(0, 2); }

double CameraModel::cy() const { return _K.at<double>(1, 2); }

const cv::Mat &CameraModel::K() const { return _K; }

const cv::Mat &CameraModel::D() const { return _D; }

const cv::Size &CameraModel::getImageSize() const { return _imageSize; }

bool CameraModel::isValid() const {
  return _K.at<double>(0, 0) > 0 && _K.at<double>(1, 1) > 0 &&
         _K.at<double>(0, 2) > 0 && _K.at<double>(1, 2) > 0;
}
