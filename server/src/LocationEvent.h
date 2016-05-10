#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include "HTTPServer.h"

class LocationEvent :
    public QEvent
{
public:
    // ownership transfer
    LocationEvent(int dbId, rtabmap::SensorData *sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo);

    int dbId() const;
    rtabmap::SensorData *sensorData() const;
    rtabmap::Transform pose() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

    Visibility _vis;

private:
    static const QEvent::Type _type;
    rtabmap::SensorData *_sensorData;
    int _dbId;
    rtabmap::Transform _pose;
    ConnectionInfo *_conInfo;
};
