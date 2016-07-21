#include "event/WordEvent.h"

const QEvent::Type WordEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
WordEvent::WordEvent(std::vector<int> wordIds, rtabmap::SensorData *sensorData, ConnectionInfo *conInfo) :
    QEvent(WordEvent::type()),
    _wordIds(std::move(wordIds)),
    _sensorData(sensorData),
    _conInfo(conInfo)
{
}

std::vector<int> WordEvent::wordIds() const
{
    return _wordIds;
}

rtabmap::SensorData *WordEvent::sensorData() const
{
    return _sensorData;
}

ConnectionInfo *WordEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type WordEvent::type()
{
    return _type;
}
