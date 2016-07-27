#include "event/ImageEvent.h"

const QEvent::Type ImageEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
ImageEvent::ImageEvent(std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData, const void *session):
    QEvent(ImageEvent::type()),
    _sensorData(std::move(sensorData)),
    _perfData(std::move(perfData)),
    _session(session)
{
}

std::unique_ptr<rtabmap::SensorData> ImageEvent::takeSensorData()
{
    return std::move(_sensorData);
}

std::unique_ptr<PerfData> ImageEvent::takePerfData()
{
    return std::move(_perfData);
}

const void *ImageEvent::getSession()
{
    return _session;
}

QEvent::Type ImageEvent::type()
{
    return _type;
}
