#include "event/SignatureEvent.h"

const QEvent::Type SignatureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

SignatureEvent::SignatureEvent(
    std::unique_ptr<std::multimap<int, cv::KeyPoint>> &&words,
    std::unique_ptr<CameraModel> &&camera,
    std::unique_ptr<std::vector<int>> &&signatureIds,
    std::unique_ptr<PerfData> &&perfData, const void *session)
    : QEvent(SignatureEvent::type()), _words(std::move(words)),
      _camera(std::move(camera)), _signatureIds(std::move(signatureIds)),
      _perfData(std::move(perfData)), _session(session) {}

std::unique_ptr<std::multimap<int, cv::KeyPoint>> SignatureEvent::takeWords() {
  return std::move(_words);
}

std::unique_ptr<CameraModel> SignatureEvent::takeCameraModel() {
  return std::move(_camera);
}

std::unique_ptr<std::vector<int>> SignatureEvent::takeSignatureIds() {
  return std::move(_signatureIds);
}

std::unique_ptr<PerfData> SignatureEvent::takePerfData() {
  return std::move(_perfData);
}

const void *SignatureEvent::getSession() { return _session; }

QEvent::Type SignatureEvent::type() { return _type; }
