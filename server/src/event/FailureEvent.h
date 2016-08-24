#pragma once

#include "data/Session.h"
#include <QEvent>
#include <memory>

class FailureEvent : public QEvent {
public:
  FailureEvent(std::unique_ptr<Session> &&session);

  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<Session> _session;
};
