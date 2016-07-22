#include "event/DetectionEvent.h"

const QEvent::Type DetectionEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

DetectionEvent::DetectionEvent(std::unique_ptr< std::vector<std::string> > &&names, const ConnectionInfo *conInfo) :
    QEvent(DetectionEvent::type()),
    _names(std::move(names)),
    _conInfo(conInfo)
{
}

std::unique_ptr< std::vector<std::string> > DetectionEvent::getNames()
{
    return std::move(_names);
}

const ConnectionInfo *DetectionEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type DetectionEvent::type()
{
    return _type;
}
