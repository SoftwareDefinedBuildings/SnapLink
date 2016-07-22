#pragma once

#include <QEvent>
#include <rtabmap/core/SensorData.h>
#include <memory>
#include "stage/HTTPServer.h"

class ImageEvent :
    public QEvent
{
public:
    // ownership transfer
    ImageEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo);

    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    ConnectionInfo *_conInfo;
};
