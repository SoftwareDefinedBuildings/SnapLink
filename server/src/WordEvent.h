#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "HTTPServer.h"

class WordEvent :
    public QEvent
{
public:
    // ownership transfer
    WordEvent(std::vector<int> wordIds, rtabmap::SensorData *sensorData, ConnectionInfo *conInfo);

    std::vector<int> wordIds() const;
    rtabmap::SensorData *sensorData() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const std::vector<int> _wordIds;
    rtabmap::SensorData *_sensorData;
    ConnectionInfo *_conInfo;
};
