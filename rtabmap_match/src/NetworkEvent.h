#pragma once

#include <QEvent>
#include "HTTPServer.h"

class NetworkEvent :
    public QEvent
{
public:
    // ownership transfer
    NetworkEvent(const std::vector<unsigned char> *payload, const ConnectionInfo *conInfo);

    const std::vector<unsigned char> *payload() const;
    const ConnectionInfo *conInfo() const;;

    static QEvent::Type type();

private:
    static QEvent::Type _type;
    const std::vector<unsigned char> *_payload;
    const ConnectionInfo *_conInfo;
};
