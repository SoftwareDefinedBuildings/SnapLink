#include "event/FailureEvent.h"

const QEvent::Type FailureEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FailureEvent::FailureEvent(std::unique_ptr<SessionInfo> &&sessionInfo) :
    QEvent(FailureEvent::type()),
    _sessionInfo(std::move(sessionInfo))
{
}

std::unique_ptr<SessionInfo> FailureEvent::takeSessionInfo()
{
    return std::move(_sessionInfo);
}

QEvent::Type FailureEvent::type()
{
    return _type;
}
