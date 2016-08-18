#include "data/SensorData.h"

SensorData::SensorData(cv::Mat &&image, CameraModel &&cameraModel)
    : _image(std::move(image)), _cameraModel(std::move(cameraModel)) {
  assert(!_image.empty());
  assert(_image.type() == CV_8UC1 || _image.type() == CV_8UC3); // Mono or RGB
}

const cv::Mat &SensorData::getImage() const { return _image; }

const CameraModel &SensorData::getCameraModel() const { return _cameraModel; }

void SensorData::setFeatures(const std::vector<cv::KeyPoint> &keypoints,
                             const cv::Mat &descriptors) {
  _keypoints = keypoints;
  _descriptors = descriptors;
}

const std::vector<cv::KeyPoint> &SensorData::keypoints() const {
  return _keypoints;
}

const cv::Mat &SensorData::descriptors() const { return _descriptors; }
