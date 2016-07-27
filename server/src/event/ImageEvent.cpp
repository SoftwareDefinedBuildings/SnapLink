#include "event/ImageEvent.h"

const QEvent::Type ImageEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
ImageEvent::ImageEvent(const void *session, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr<PerfData> &&perfData):
    QEvent(ImageEvent::type()),
    _session(session),
    _sensorData(std::move(sensorData)),
    _perfData(std::move(PerfData))
{
}

const void *ImageEvent::getSession()
{
    return _session;
}

std::unique_ptr<rtabmap::SensorData> ImageEvent::takeSensorData()
{
    return std::move(_sensorData);
}

std::unique_ptr<PerfData> ImageEvent::takePerfData();
{
    return std::move(_perfData);
}

QEvent::Type ImageEvent::type()
{
    return _type;
}
