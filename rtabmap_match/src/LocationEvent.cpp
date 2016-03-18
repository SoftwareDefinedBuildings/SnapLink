#include "LocationEvent.h"

QEvent::Type LocationEvent::_type = QEvent::None;

// ownership transfer
LocationEvent::LocationEvent(rtabmap::SensorData *sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo) :
    QEvent(LocationEvent::type()),
    _sensorData(sensorData),
    _pose(pose),
    _conInfo(conInfo)
{
}

rtabmap::SensorData *LocationEvent::sensorData() const
{
    return _sensorData;
}

rtabmap::Transform LocationEvent::pose() const
{
    return _pose;
}

ConnectionInfo *LocationEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type LocationEvent::type()
{
    if (_type == QEvent::None)
    {
        _type = static_cast<QEvent::Type>(QEvent::registerEventType());
    }
    return _type;
}
