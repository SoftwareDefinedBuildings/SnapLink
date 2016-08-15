#pragma once

#include "data/PerfData.h"
#include "data/SensorData.h"
#include "data/Signature.h"
#include <QEvent>

class SignatureEvent : public QEvent {
public:
  // ownership transfer
  SignatureEvent(std::unique_ptr<std::vector<int>> &&wordIds,
                 std::unique_ptr<SensorData> &&sensorData,
                 std::vector<std::unique_ptr<Signature>> &&signatures,
                 std::unique_ptr<PerfData> &&perfData, const void *session);

  std::unique_ptr<std::vector<int>> takeWordIds();
  std::unique_ptr<SensorData> takeSensorData();
  std::vector<std::unique_ptr<Signature>> takeSignatures();
  std::unique_ptr<PerfData> takePerfData();
  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::vector<int>> _wordIds;
  std::unique_ptr<SensorData> _sensorData;
  std::vector<std::unique_ptr<Signature>> _signatures;
  std::unique_ptr<PerfData> _perfData;
  const void *_session;
};
