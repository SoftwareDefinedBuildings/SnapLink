#pragma once

#include <QEvent>
#include <memory>
#include "data/PerfData.h"

class FailureEvent :
    public QEvent
{
public:
    FailureEvent(const void *session = nullptr);

    const void *getSession();

    static QEvent::Type type();

private:
    static const QEvent::Type _type;
    const void *_session;
};
