#include "event/NetworkEvent.h"

const QEvent::Type NetworkEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
NetworkEvent::NetworkEvent(SessionInfo *sessionInfo) :
    QEvent(NetworkEvent::type()),
    _sessionInfo(sessionInfo)
{
}

SessionInfo *NetworkEvent::sessionInfo() const
{
    return _sessionInfo;
}

QEvent::Type NetworkEvent::type()
{
    return _type;
}
