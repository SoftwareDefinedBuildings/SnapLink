#include "event/SignatureEvent.h"

const QEvent::Type SignatureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
SignatureEvent::SignatureEvent(std::unique_ptr< std::vector<int> > &&wordIds, std::unique_ptr<rtabmap::SensorData> &&sensorData, std::unique_ptr< std::vector<Signature *> > &&signatures, ConnectionInfo *conInfo) :
    QEvent(SignatureEvent::type()),
    _wordIds(std::move(wordIds)),
    _sensorData(std::move(sensorData)),
    _signatures(std::move(signatures)),
    _conInfo(conInfo)
{
}

std::unique_ptr< std::vector<int> > SignatureEvent::takeWordIds()
{
    return std::move(_wordIds);
}

std::unique_ptr<rtabmap::SensorData> SignatureEvent::takeSensorData()
{
    return std::move(_sensorData);
}

std::unique_ptr< std::vector<Signature *> > SignatureEvent::takeSignatures()
{
    return std::move(_signatures);
}

ConnectionInfo *SignatureEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type SignatureEvent::type()
{
    return _type;
}
