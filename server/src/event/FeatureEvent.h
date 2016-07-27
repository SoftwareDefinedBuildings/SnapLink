#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include <memory>
#include "data/PerfData.h"

class FeatureEvent :
    public QEvent
{
public:
    // ownership transfer
    FeatureEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData, const void *session = NULL);

    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::unique_ptr<PerfData> _perfData;
};
