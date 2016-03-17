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
    LocationEvent(const rtabmap::SensorData *sensorData, const rtabmap::Transform *pose, const ConnectionInfo *conInfo);

    const rtabmap::SensorData *sensorData() const;
    const rtabmap::Transform *pose() const;
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

    Visibility _vis;

private:
    static QEvent::Type _type;
    const rtabmap::SensorData *_sensorData;
    const rtabmap::Transform *_pose;
    const ConnectionInfo *_conInfo;
};
