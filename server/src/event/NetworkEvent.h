#pragma once

#include <QEvent>
#include "data/SessionInfo.h"

class NetworkEvent :
    public QEvent
{
public:
    // ownership transfer
    NetworkEvent(SessionInfo *sessionInfo);

    SessionInfo *sessionInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    SessionInfo *_sessionInfo;
};
