#include "event/FailureEvent.h"

const QEvent::Type FailureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

// ownership transfer
FailureEvent::FailureEvent(const void *session)
    : QEvent(FailureEvent::type()), _session(session) {}

const void *FailureEvent::getSession() { return _session; }

QEvent::Type FailureEvent::type() { return _type; }
