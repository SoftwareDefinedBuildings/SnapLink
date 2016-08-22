#include "event/QueryEvent.h"

const QEvent::Type QueryEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

QueryEvent::QueryEvent(std::unique_ptr<cv::Mat> &&image,
                       std::unique_ptr<CameraModel> &&camera,
                       std::unique_ptr<PerfData> &&perfData,
                       const void *session)
    : QEvent(QueryEvent::type()), _image(std::move(image)),
      _camera(std::move(camera)), _perfData(std::move(perfData)),
      _session(session) {}

std::unique_ptr<cv::Mat> QueryEvent::takeImage() { return std::move(_image); }

std::unique_ptr<CameraModel> QueryEvent::takeCameraModel() {
  return std::move(_camera);
}

std::unique_ptr<PerfData> QueryEvent::takePerfData() {
  return std::move(_perfData);
}

const void *QueryEvent::getSession() { return _session; }

QEvent::Type QueryEvent::type() { return _type; }
