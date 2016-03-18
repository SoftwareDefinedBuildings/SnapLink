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
    LocationEvent(rtabmap::SensorData *sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo);

    rtabmap::SensorData *sensorData() const;
    rtabmap::Transform pose() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

    Visibility _vis;

private:
    static QEvent::Type _type;
    rtabmap::SensorData *_sensorData;
    rtabmap::Transform _pose;
    ConnectionInfo *_conInfo;
};
