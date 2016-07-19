#include "event/ImageEvent.h"

const QEvent::Type ImageEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
ImageEvent::ImageEvent(rtabmap::SensorData *sensorData, ConnectionInfo *conInfo) :
    QEvent(ImageEvent::type()),
    _sensorData(sensorData),
    _conInfo(conInfo)
{
}

rtabmap::SensorData *ImageEvent::sensorData() const
{
    return _sensorData;
}

ConnectionInfo *ImageEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type ImageEvent::type()
{
    return _type;
}
