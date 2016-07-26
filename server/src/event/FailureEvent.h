#pragma once

#include <QEvent>
#include <memory>
#include "data/SessionInfo.h"

class FailureEvent :
    public QEvent
{
public:
    FailureEvent(std::unique_ptr<SessionInfo> &&sessionInfo);

    std::unique_ptr<SessionInfo> takeSessionInfo();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr<SessionInfo> _sessionInfo;
};
