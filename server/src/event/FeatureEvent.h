#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "data/SessionInfo.h"

class FeatureEvent :
    public QEvent
{
public:
    // ownership transfer
    FeatureEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, SessionInfo *sessionInfo);

    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    SessionInfo *sessionInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    SessionInfo *_sessionInfo;
};
