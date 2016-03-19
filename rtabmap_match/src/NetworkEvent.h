#pragma once

#include <QEvent>
#include "HTTPServer.h"

class NetworkEvent :
    public QEvent
{
public:
    // ownership transfer
    NetworkEvent(std::vector<unsigned char> *payload, ConnectionInfo *conInfo);

    std::vector<unsigned char> *payload() const;
    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    std::vector<unsigned char> *_payload;
    ConnectionInfo *_conInfo;
};
