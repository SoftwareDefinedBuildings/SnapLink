#include "NetworkEvent.h"

const QEvent::Type NetworkEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
NetworkEvent::NetworkEvent(ConnectionInfo *conInfo) :
    QEvent(NetworkEvent::type()),
    _conInfo(conInfo)
{
}

ConnectionInfo *NetworkEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type NetworkEvent::type()
{
    return _type;
}
