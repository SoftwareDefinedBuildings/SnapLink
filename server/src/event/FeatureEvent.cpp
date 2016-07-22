#include "event/FeatureEvent.h"

const QEvent::Type FeatureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FeatureEvent::FeatureEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo) :
    QEvent(FeatureEvent::type()),
    _sensorData(std::move(sensorData)),
    _conInfo(conInfo)
{
}

std::unique_ptr<rtabmap::SensorData> FeatureEvent::takeSensorData()
{
    return std::move(_sensorData);
}

ConnectionInfo *FeatureEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type FeatureEvent::type()
{
    return _type;
}
