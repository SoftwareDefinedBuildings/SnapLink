#pragma once

#include <rtabmap/core/SensorData.h>
#include <QEvent>
#include "data/PerfData.h"

class WordEvent :
    public QEvent
{
public:
    WordEvent(std::unique_ptr< std::vector<int> > &&wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData, const void *session = nullptr);

    std::unique_ptr<std::vector<int>> takeWordIds();
    std::unique_ptr<rtabmap::SensorData> takeSensorData();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr< std::vector<int> > _wordIds;
    std::unique_ptr<rtabmap::SensorData> _sensorData;
    std::unique_ptr<PerfData> _perfData;
};
