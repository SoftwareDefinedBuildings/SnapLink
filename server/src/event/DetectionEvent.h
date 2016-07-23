#pragma once

#include <QEvent>
#include <vector>
#include <memory>
#include "data/SessionInfo.h"

class DetectionEvent :
    public QEvent
{
public:
    DetectionEvent(std::unique_ptr< std::vector<std::string> > &&names, std::unique_ptr<SessionInfo> &&sessionInfo);

    std::unique_ptr< std::vector<std::string> > takeNames();
    std::unique_ptr<SessionInfo> takeSessionInfo();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::unique_ptr< std::vector<std::string> > _names;
    std::unique_ptr<SessionInfo> _sessionInfo;
};
