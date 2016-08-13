#pragma once

#include "data/PerfData.h"
#include <QEvent>
#include <memory>

class FailureEvent : public QEvent {
public:
  FailureEvent(const void *session);

  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  const void *_session;
};
