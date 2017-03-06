#include "run/event/DetectionEvent.h"

const QEvent::Type DetectionEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

DetectionEvent::DetectionEvent(
    std::unique_ptr<std::vector<std::string>> &&results,
    std::unique_ptr<Session> &&session)
    : QEvent(DetectionEvent::type()), _results(std::move(results)),
      _session(std::move(session)) {}

std::unique_ptr<std::vector<std::string>> DetectionEvent::takeResults() {
  return std::move(_results);
}

std::unique_ptr<Session> DetectionEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type DetectionEvent::type() { return _type; }
