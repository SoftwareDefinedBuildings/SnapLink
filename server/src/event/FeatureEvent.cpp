#include "event/FeatureEvent.h"

const QEvent::Type FeatureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FeatureEvent::FeatureEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, SessionInfo *sessionInfo) :
    QEvent(FeatureEvent::type()),
    _sensorData(std::move(sensorData)),
    _sessionInfo(sessionInfo)
{
}

std::unique_ptr<rtabmap::SensorData> FeatureEvent::takeSensorData()
{
    return std::move(_sensorData);
}

SessionInfo *FeatureEvent::sessionInfo() const
{
    return _sessionInfo;
}

QEvent::Type FeatureEvent::type()
{
    return _type;
}
