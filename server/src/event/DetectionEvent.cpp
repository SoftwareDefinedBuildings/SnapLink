#include "event/DetectionEvent.h"

const QEvent::Type DetectionEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

DetectionEvent::DetectionEvent(std::unique_ptr< std::vector<std::string> > &&names, std::unique_ptr<SessionInfo> &&sessionInfo) :
    QEvent(DetectionEvent::type()),
    _names(std::move(names)),
    _sessionInfo(std::move(sessionInfo))
{
}

std::unique_ptr< std::vector<std::string> > DetectionEvent::getNames()
{
    return std::move(_names);
}

std::unique_ptr<SessionInfo> DetectionEvent::takeSessionInfo()
{
    return std::move(_sessionInfo);
}

QEvent::Type DetectionEvent::type()
{
    return _type;
}
