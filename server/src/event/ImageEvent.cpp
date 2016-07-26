#include "event/ImageEvent.h"

const QEvent::Type ImageEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
ImageEvent::ImageEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, SessionInfo *sessionInfo) :
    QEvent(ImageEvent::type()),
    _sensorData(std::move(sensorData)),
    _sessionInfo(sessionInfo)
{
}

std::unique_ptr<rtabmap::SensorData> ImageEvent::takeSensorData()
{
    return std::move(_sensorData);
}

SessionInfo *ImageEvent::sessionInfo() const
{
    return _sessionInfo;
}

QEvent::Type ImageEvent::type()
{
    return _type;
}
