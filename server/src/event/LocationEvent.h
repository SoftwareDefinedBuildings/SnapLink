#pragma once

#include "data/PerfData.h"
#include "data/SensorData.h"
#include "data/Transform.h"
#include <QEvent>
#include <memory>

class LocationEvent : public QEvent {
public:
  // ownership transfer
  LocationEvent(int dbId, std::unique_ptr<SensorData> &&sensorData,
                std::unique_ptr<Transform> &&pose,
                std::unique_ptr<PerfData> &&perfData, const void *session);

  int dbId() const;
  std::unique_ptr<SensorData> takeSensorData();
  std::unique_ptr<Transform> takePose();
  std::unique_ptr<PerfData> takePerfData();
  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  int _dbId;
  std::unique_ptr<SensorData> _sensorData;
  std::unique_ptr<Transform> _pose;
  std::unique_ptr<PerfData> _perfData;
  const void *_session;
};
