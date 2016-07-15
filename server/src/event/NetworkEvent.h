#pragma once

#include <QEvent>
#include "stage/HTTPServer.h"

class NetworkEvent :
    public QEvent
{
public:
    // ownership transfer
    NetworkEvent(ConnectionInfo *conInfo);

    ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    ConnectionInfo *_conInfo;
};
