#include "event/DetectionEvent.h"

const QEvent::Type DetectionEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

DetectionEvent::DetectionEvent(
    std::unique_ptr<std::vector<std::string>> &&names,
    std::unique_ptr<PerfData> &&perfData, const void *session)
    : QEvent(DetectionEvent::type()), _names(std::move(names)),
      _perfData(std::move(perfData)), _session(session) {}

std::unique_ptr<std::vector<std::string>> DetectionEvent::takeNames() {
  return std::move(_names);
}

std::unique_ptr<PerfData> DetectionEvent::takePerfData() {
  return std::move(_perfData);
}

const void *DetectionEvent::getSession() { return _session; }

QEvent::Type DetectionEvent::type() { return _type; }
