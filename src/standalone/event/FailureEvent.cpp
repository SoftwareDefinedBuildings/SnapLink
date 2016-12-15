#include "event/FailureEvent.h"

const QEvent::Type FailureEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

FailureEvent::FailureEvent(std::unique_ptr<Session> &&session)
    : QEvent(FailureEvent::type()), _session(std::move(session)) {}

std::shared_ptr<Session> FailureEvent::takeSession() {
  std::shared_ptr<Session> sesion = _session;
  _session.reset();
  return session;
}

QEvent::Type FailureEvent::type() { return _type; }
