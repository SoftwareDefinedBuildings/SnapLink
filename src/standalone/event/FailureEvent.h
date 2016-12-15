#pragma once

#include "data/Session.h"
#include <QEvent>
#include <memory>

class FailureEvent : public QEvent {
public:
  FailureEvent(std::shared_ptr<Session> session);

  std::shared_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::shared_ptr<Session> _session;
};
