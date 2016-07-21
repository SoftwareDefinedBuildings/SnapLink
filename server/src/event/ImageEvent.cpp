#include "event/ImageEvent.h"

const QEvent::Type ImageEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
ImageEvent::ImageEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, ConnectionInfo *conInfo) :
    QEvent(ImageEvent::type()),
    _sensorData(std::move(sensorData)),
    _conInfo(conInfo)
{
}

std::unique_ptr<rtabmap::SensorData> ImageEvent::getSensorData()
{
    return std::move(_sensorData);
}

ConnectionInfo *ImageEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type ImageEvent::type()
{
    return _type;
}
