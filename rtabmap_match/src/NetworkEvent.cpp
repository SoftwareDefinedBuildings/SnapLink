#include "NetworkEvent.h"

QEvent::Type NetworkEvent::_type = QEvent::None;

// ownership transfer
NetworkEvent::NetworkEvent(const std::vector<unsigned char> *payload, const ConnectionInfo *conInfo) :
    QEvent(NetworkEvent::type()),
    _payload(payload),
    _conInfo(conInfo)
{
}

const std::vector<unsigned char> *NetworkEvent::payload() const
{
    return _payload;
}

const ConnectionInfo *NetworkEvent::conInfo() const
{
    return _conInfo;
}

QEvent::Type NetworkEvent::type()
{
    if (_type == QEvent::None)
    {
        _type = static_cast<QEvent::Type>(QEvent::registerEventType());
    }
    return _type;
}
