#include "event/LocationEvent.h"

const QEvent::Type LocationEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
LocationEvent::LocationEvent(int dbId, std::unique_ptr<SensorData> &&sensorData,
                             std::unique_ptr<Transform> &&pose,
                             std::unique_ptr<PerfData> &&perfData,
                             const void *session)
    : QEvent(LocationEvent::type()), _dbId(dbId),
      _sensorData(std::move(sensorData)), _pose(std::move(pose)),
      _perfData(std::move(perfData)), _session(session) {}

int LocationEvent::dbId() const { return _dbId; }

std::unique_ptr<SensorData> LocationEvent::takeSensorData() {
  return std::move(_sensorData);
}

std::unique_ptr<Transform> LocationEvent::takePose() {
  return std::move(_pose);
}

std::unique_ptr<PerfData> LocationEvent::takePerfData() {
  return std::move(_perfData);
}

const void *LocationEvent::getSession() { return _session; }

QEvent::Type LocationEvent::type() { return _type; }
