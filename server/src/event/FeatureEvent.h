#pragma once

#include <QEvent>
#include <memory>
#include "data/PerfData.h"
#include "data/SensorData.h"

class FeatureEvent :
    public QEvent
{
public:
    // ownership transfer
    FeatureEvent(std::unique_ptr<SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData, const void *session);

    std::unique_ptr<SensorData> takeSensorData();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr<SensorData> _sensorData;
    std::unique_ptr<PerfData> _perfData;
};
