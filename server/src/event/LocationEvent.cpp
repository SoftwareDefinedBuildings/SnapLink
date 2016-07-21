#include "event/LocationEvent.h"

const QEvent::Type LocationEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
LocationEvent::LocationEvent(int dbId, std::unique_ptr<rtabmap::SensorData> &&sensorData, rtabmap::Transform pose, ConnectionInfo *conInfo) :
    QEvent(LocationEvent::type()),
    _dbId(dbId),
    _sensorData(std::move(sensorData)),
    _pose(std::move(pose)),
    _conInfo(conInfo)
{
}

int LocationEvent::dbId() const
{
    return _dbId;
}

std::unique_ptr<rtabmap::SensorData> LocationEvent::getSensorData()
{
    return std::move(_sensorData);
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
