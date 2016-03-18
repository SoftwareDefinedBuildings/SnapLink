#include "FailureEvent.h"

QEvent::Type FailureEvent::_type = QEvent::None;

// ownership transfer
FailureEvent::FailureEvent(const ConnectionInfo *conInfo) :
    QEvent(FailureEvent::type()),
    _conInfo(conInfo)
{
}

const ConnectionInfo *FailureEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type FailureEvent::type()
{
    if (_type == QEvent::None)
    {
        _type = static_cast<QEvent::Type>(QEvent::registerEventType());
    }
    return _type;
}
