#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "stage/HTTPServer.h"

class WordEvent :
    public QEvent
{
public:
    WordEvent(std::unique_ptr< std::vector<int> > &&wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo);

    std::unique_ptr<std::vector<int>> takeWordIds();
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr< std::vector<int> > _wordIds;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    ConnectionInfo *_conInfo;
};
