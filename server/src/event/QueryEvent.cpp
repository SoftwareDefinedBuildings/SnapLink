#include "event/QueryEvent.h"

const QEvent::Type QueryEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
QueryEvent::QueryEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData, const void *session):
    QEvent(QueryEvent::type()),
    _sensorData(std::move(sensorData)),
    _perfData(std::move(perfData)),
    _session(session)
{
}

std::unique_ptr<rtabmap::SensorData> QueryEvent::takeSensorData()
{
    return std::move(_sensorData);
}

std::unique_ptr<PerfData> QueryEvent::takePerfData()
{
    return std::move(_perfData);
}

const void *QueryEvent::getSession()
{
    return _session;
}

QEvent::Type QueryEvent::type()
{
    return _type;
}
