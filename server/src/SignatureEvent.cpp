#include "SignatureEvent.h"

const QEvent::Type SignatureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
SignatureEvent::SignatureEvent(std::vector<int> wordIds, rtabmap::SensorData *sensorData, std::vector<Signature *> signatures, ConnectionInfo *conInfo) :
    QEvent(SignatureEvent::type()),
    _wordIds(wordIds),
    _sensorData(sensorData),
    _signatures(signatures),
    _conInfo(conInfo)
{
}

std::vector<int> SignatureEvent::wordIds() const
{
    return _wordIds;
}

rtabmap::SensorData *SignatureEvent::sensorData() const
{
    return _sensorData;
}

std::vector<Signature *> SignatureEvent::signatures() const
{
    return _signatures;
}

ConnectionInfo *SignatureEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type SignatureEvent::type()
{
    return _type;
}
