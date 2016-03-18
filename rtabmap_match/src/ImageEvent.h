#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include "HTTPServer.h"

class ImageEvent :
    public QEvent
{
public:
    // ownership transfer
    ImageEvent(rtabmap::SensorData *sensorData, ConnectionInfo *conInfo);

    rtabmap::SensorData *sensorData() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static QEvent::Type _type;
    rtabmap::SensorData *_sensorData;
    ConnectionInfo *_conInfo;
};
