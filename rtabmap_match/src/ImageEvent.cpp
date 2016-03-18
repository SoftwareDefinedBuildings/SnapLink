#include "ImageEvent.h"

// TODO init here
QEvent::Type ImageEvent::_type = QEvent::None;

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
    if (_type == QEvent::None)
    {
        _type = static_cast<QEvent::Type>(QEvent::registerEventType());
    }
    return _type;
}
