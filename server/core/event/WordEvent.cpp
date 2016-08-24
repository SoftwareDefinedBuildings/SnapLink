#include "event/WordEvent.h"

const QEvent::Type WordEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

WordEvent::WordEvent(std::unique_ptr<std::vector<int>> &&wordIds,
                     std::unique_ptr<std::vector<cv::KeyPoint>> &&keyPoints,
                     std::unique_ptr<CameraModel> &&camera,
                     std::unique_ptr<Session> &&session)
    : QEvent(WordEvent::type()), _wordIds(std::move(wordIds)),
      _keyPoints(std::move(keyPoints)), _camera(std::move(camera)),
      _session(std::move(session)) {}

std::unique_ptr<std::vector<int>> WordEvent::takeWordIds() {
  return std::move(_wordIds);
}

std::unique_ptr<std::vector<cv::KeyPoint>> WordEvent::takeKeyPoints() {
  return std::move(_keyPoints);
}

std::unique_ptr<CameraModel> WordEvent::takeCameraModel() {
  return std::move(_camera);
}

std::unique_ptr<Session> WordEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type WordEvent::type() { return _type; }
