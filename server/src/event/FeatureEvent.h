#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "stage/HTTPServer.h"

class FeatureEvent :
    public QEvent
{
public:
    // ownership transfer
    FeatureEvent(rtabmap::SensorData *sensorData, ConnectionInfo *conInfo);

    rtabmap::SensorData *sensorData() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    rtabmap::SensorData *_sensorData;
    ConnectionInfo *_conInfo;
};
