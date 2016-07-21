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
    LocationEvent(int dbId, std::unique_ptr<rtabmap::SensorData> &&sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo);

    int dbId() const;
    std::unique_ptr<rtabmap::SensorData> getSensorData();
    rtabmap::Transform pose() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

    Visibility _vis;

private:
    static const QEvent::Type _type;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    int _dbId;
    rtabmap::Transform _pose;
    ConnectionInfo *_conInfo;
};
