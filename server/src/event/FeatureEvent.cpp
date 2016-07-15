#include "event/FeatureEvent.h"

const QEvent::Type FeatureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FeatureEvent::FeatureEvent(rtabmap::SensorData *sensorData, ConnectionInfo *conInfo) :
    QEvent(FeatureEvent::type()),
    _sensorData(sensorData),
    _conInfo(conInfo)
{
}

rtabmap::SensorData *FeatureEvent::sensorData() const
{
    return _sensorData;
}

ConnectionInfo *FeatureEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type FeatureEvent::type()
{
    return _type;
}
