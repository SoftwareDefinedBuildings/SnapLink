#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "stage/HTTPServer.h"

class WordEvent :
    public QEvent
{
public:
    // ownership transfer
    WordEvent(std::vector<int> wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo);

    std::vector<int> wordIds() const;
    std::unique_ptr<rtabmap::SensorData> getSensorData();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const std::vector<int> _wordIds;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    ConnectionInfo *_conInfo;
};
