#include "message/FeatureEvent.h"

const QEvent::Type FeatureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

FeatureEvent::FeatureEvent(
    std::unique_ptr<std::vector<cv::KeyPoint>> &&keyPoints,
    std::unique_ptr<cv::Mat> &&descriptors,
    std::unique_ptr<CameraModel> &&camera, std::unique_ptr<Session> &&session)
    : QEvent(FeatureEvent::type()), _keyPoints(std::move(keyPoints)),
      _descriptors(std::move(descriptors)), _camera(std::move(camera)),
      _session(std::move(session)) {}

std::unique_ptr<std::vector<cv::KeyPoint>> FeatureEvent::takeKeyPoints() {
  return std::move(_keyPoints);
}

std::unique_ptr<cv::Mat> FeatureEvent::takeDescriptors() {
  return std::move(_descriptors);
}

std::unique_ptr<CameraModel> FeatureEvent::takeCameraModel() {
  return std::move(_camera);
}

std::unique_ptr<Session> FeatureEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type FeatureEvent::type() { return _type; }
