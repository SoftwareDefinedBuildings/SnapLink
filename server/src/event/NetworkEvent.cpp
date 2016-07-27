#include "event/NetworkEvent.h"

const QEvent::Type NetworkEvent::_type = static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
NetworkEvent::NetworkEvent(std::unique_ptr< std::vector<char> > &&rawData, std::unique_ptr<PerfData> &&perfData, const void *session) :
    QEvent(NetworkEvent::type()),
    _rawData(std::move(rawData)),
    _perfData(std::move(PerfData)),
    _session(session)
{
}

std::unique_ptr< std::vector<char> > NetworkEvent::takeRawData()
{
    return std::move(_rawData);
}

std::unique_ptr<PerfData> NetworkEvent::takePerfData()
{
    return std::move(_perfData);
}

const void *NetworkEvent::getSession()
{
    return _session;
}

QEvent::Type NetworkEvent::type()
{
    return _type;
}
