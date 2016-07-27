#pragma once

#include <QEvent>
#include <memory>
#include "data/PerfData.h"

class NetworkEvent :
    public QEvent
{
public:
    NetworkEvent(std::unique_ptr< std::vector<char> > &&rawData, std::unique_ptr<PerfData> &&perfData, const void *session = nullptr);

    std::unique_ptr< std::vector<char> > takeRawData();
    std::unique_ptr<PerfData> takePerfData();
    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
    std::unique_ptr< std::vector<char> > _rawData;
    std::unique_ptr<PerfData> _perfData;
};
