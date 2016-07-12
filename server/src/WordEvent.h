#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "HTTPServer.h"

class WordEvent :
    public QEvent
{
public:
    // ownership transfer
    WordEvent(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, const ConnectionInfo *conInfo);

    const std::vector<int> wordIds() const;
    const rtabmap::SensorData *sensorData() const;
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const std::vector<int> _wordIds;
    const rtabmap::SensorData *_sensorData;
    const ConnectionInfo *_conInfo;
};
