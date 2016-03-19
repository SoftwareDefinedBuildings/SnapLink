#include "NetworkEvent.h"

const QEvent::Type NetworkEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
NetworkEvent::NetworkEvent(std::vector<unsigned char> *payload, ConnectionInfo *conInfo) :
    QEvent(NetworkEvent::type()),
    _payload(payload),
    _conInfo(conInfo)
{
}

std::vector<unsigned char> *NetworkEvent::payload() const
{
    return _payload;
}

ConnectionInfo *NetworkEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type NetworkEvent::type()
{
    return _type;
}
