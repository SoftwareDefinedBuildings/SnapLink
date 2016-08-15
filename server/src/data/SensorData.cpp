#include "data/SensorData.h"

SensorData::SensorData() = default;

SensorData::SensorData(const cv::Mat &image, const cv::Mat &depth,
                       CameraModel &&cameraModel, int id, double stamp)
    : _id(id), _stamp(stamp), _image(image), _depth(depth),
      _cameraModel(std::move(cameraModel)) {
  assert(!image.empty());
  assert(image.type() == CV_8UC1 || image.type() == CV_8UC3); // Mono or RGB
}

SensorData::~SensorData() = default;

int SensorData::getId() const { return _id; }

void SensorData::setId(int id) { _id = id; }

double SensorData::getStamp() const { return _stamp; }

const cv::Mat &SensorData::getImage() const { return _image; }

const cv::Mat &SensorData::getDepth() const { return _depth; }

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
