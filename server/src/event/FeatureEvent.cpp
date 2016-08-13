#include "event/FeatureEvent.h"

const QEvent::Type FeatureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FeatureEvent::FeatureEvent(std::unique_ptr<SensorData> &&sensorData,
                           std::unique_ptr<PerfData> &&perfData,
                           const void *session)
    : QEvent(FeatureEvent::type()), _sensorData(std::move(sensorData)),
      _perfData(std::move(perfData)), _session(session) {}

std::unique_ptr<SensorData> FeatureEvent::takeSensorData() {
  return std::move(_sensorData);
}

std::unique_ptr<PerfData> FeatureEvent::takePerfData() {
  return std::move(_perfData);
}

const void *FeatureEvent::getSession() { return _session; }

QEvent::Type FeatureEvent::type() { return _type; }
