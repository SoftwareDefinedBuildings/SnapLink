#include "event/WordEvent.h"

const QEvent::Type WordEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

WordEvent::WordEvent(std::unique_ptr<std::vector<int>> &&wordIds,
                     std::unique_ptr<SensorData> &&sensorData,
                     std::unique_ptr<PerfData> &&perfData, const void *session)
    : QEvent(WordEvent::type()), _wordIds(std::move(wordIds)),
      _sensorData(std::move(sensorData)), _perfData(std::move(perfData)),
      _session(session) {}

std::unique_ptr<std::vector<int>> WordEvent::takeWordIds() {
  return std::move(_wordIds);
}

std::unique_ptr<SensorData> WordEvent::takeSensorData() {
  return std::move(_sensorData);
}

std::unique_ptr<PerfData> WordEvent::takePerfData() {
  return std::move(_perfData);
}

const void *WordEvent::getSession() { return _session; }

QEvent::Type WordEvent::type() { return _type; }
