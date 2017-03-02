#include "run/event/FailureEvent.h"

const QEvent::Type FailureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

FailureEvent::FailureEvent(std::unique_ptr<Session> &&session)
    : QEvent(FailureEvent::type()), _session(std::move(session)) {}

std::unique_ptr<Session> FailureEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type FailureEvent::type() { return _type; }
