#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include "HTTPServer.h"

class ImageEvent :
    public QEvent
{
public:
    // ownership transfer
    ImageEvent(const rtabmap::SensorData *sensorData, const ConnectionInfo *conInfo);

    const rtabmap::SensorData *sensorData() const;
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static QEvent::Type _type;
    const rtabmap::SensorData *_sensorData;
    const ConnectionInfo *_conInfo;
};
