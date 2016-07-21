#include "event/WordEvent.h"

const QEvent::Type WordEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
WordEvent::WordEvent(std::vector<int> wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo) :
    QEvent(WordEvent::type()),
    _wordIds(std::move(wordIds)),
    _sensorData(std::move(sensorData)),
    _conInfo(conInfo)
{
}

std::vector<int> WordEvent::wordIds() const
{
    return _wordIds;
}

std::unique_ptr<rtabmap::SensorData> WordEvent::getSensorData()
{
    return std::move(_sensorData);
}

ConnectionInfo *WordEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type WordEvent::type()
{
    return _type;
}
