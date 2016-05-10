#include "LocationEvent.h"

const QEvent::Type LocationEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
LocationEvent::LocationEvent(int dbId, rtabmap::SensorData *sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo) :
    QEvent(LocationEvent::type()),
    _dbId(dbId),
    _sensorData(sensorData),
    _pose(pose),
    _conInfo(conInfo)
{
}

int LocationEvent::dbId() const
{
    return _dbId;
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
    return _type;
}
