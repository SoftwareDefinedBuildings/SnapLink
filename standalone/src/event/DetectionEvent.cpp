#include "event/DetectionEvent.h"

const QEvent::Type DetectionEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

DetectionEvent::DetectionEvent(
    std::unique_ptr<std::vector<std::string>> &&names,
    std::unique_ptr<Session> &&session)
    : QEvent(DetectionEvent::type()), _names(std::move(names)),
      _session(std::move(session)) {}

std::unique_ptr<std::vector<std::string>> DetectionEvent::takeNames() {
  return std::move(_names);
}

std::unique_ptr<Session> DetectionEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type DetectionEvent::type() { return _type; }
