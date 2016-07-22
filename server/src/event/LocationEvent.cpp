#include "event/LocationEvent.h"

const QEvent::Type LocationEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
LocationEvent::LocationEvent(int dbId, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<rtabmap::Transform> &&pose, ConnectionInfo *conInfo) :
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

std::unique_ptr<rtabmap::SensorData> LocationEvent::takeSensorData()
{
    return std::move(_sensorData);
}

std::unique_ptr<rtabmap::Transform> LocationEvent::takePose()
{
    return std::move(_pose);
}

ConnectionInfo *LocationEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type LocationEvent::type()
{
    return _type;
}
