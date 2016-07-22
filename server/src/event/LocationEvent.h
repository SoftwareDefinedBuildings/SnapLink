#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include "stage/HTTPServer.h"

class LocationEvent :
    public QEvent
{
public:
    // ownership transfer
    LocationEvent(int dbId, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<rtabmap::Transform> &&pose, ConnectionInfo *conInfo);

    int dbId() const;
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::unique_ptr<rtabmap::Transform> takePose();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

    Visibility _vis;

private:
    static const QEvent::Type _type;
    int _dbId;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::unique_ptr<rtabmap::Transform> _pose;
    ConnectionInfo *_conInfo;
};
