#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "stage/HTTPServer.h"

class FeatureEvent :
    public QEvent
{
public:
    // ownership transfer
    FeatureEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo);

    std::unique_ptr<rtabmap::SensorData> getSensorData();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    ConnectionInfo *_conInfo;
};
