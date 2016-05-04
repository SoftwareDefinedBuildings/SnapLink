#pragma once

#include <QEvent>
#include "HTTPServer.h"

class FailureEvent :
    public QEvent
{
public:
    // ownership transfer
    FailureEvent(const ConnectionInfo *conInfo);

    // get the names of the detected objects
    const ConnectionInfo *conInfo() const;

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const ConnectionInfo *_conInfo;
};
