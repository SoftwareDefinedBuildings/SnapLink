#include "WordEvent.h"

const QEvent::Type WordEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
WordEvent::WordEvent(std::vector<int> wordIds, const rtabmap::SensorData *sensorData, const ConnectionInfo *conInfo) :
    QEvent(WordEvent::type()),
    _wordIds(wordIds),
    _sensorData(sensorData),
    _conInfo(conInfo)
{
}

const std::vector<int> WordEvent::wordIds() const
{
    return &_wordIds;
}

const rtabmap::SensorData *WordEvent::sensorData() const
{
    return _sensorData;
}

const ConnectionInfo *WordEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type WordEvent::type()
{
    return _type;
}
