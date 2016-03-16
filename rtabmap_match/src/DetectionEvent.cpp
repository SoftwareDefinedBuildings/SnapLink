#include "DetectionEvent.h"

QEvent::Type DetectionEvent::_type = QEvent::None;

// ownership transfer
DetectionEvent::DetectionEvent(const std::vector<std::string> *names, const ConnectionInfo *conInfo) :
    QEvent(DetectionEvent::type()),
    _names(names),
    _conInfo(conInfo)
{
}

// get the names of the detected objects
const std::vector<std::string> *DetectionEvent::names() const
{
    return _names;
}

const ConnectionInfo *DetectionEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type DetectionEvent::type()
{
    if (_type == QEvent::None)
    {
        _type = static_cast<QEvent::Type>(QEvent::registerEventType());
    }
    return _type;
}
