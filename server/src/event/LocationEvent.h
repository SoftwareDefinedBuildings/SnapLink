#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include "data/SessionInfo.h"

class LocationEvent :
    public QEvent
{
public:
    // ownership transfer
    LocationEvent(int dbId, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<rtabmap::Transform> &&pose, SessionInfo *sessionInfo);

    int dbId() const;
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::unique_ptr<rtabmap::Transform> takePose();
    SessionInfo *sessionInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    int _dbId;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::unique_ptr<rtabmap::Transform> _pose;
    SessionInfo *_sessionInfo;
};
