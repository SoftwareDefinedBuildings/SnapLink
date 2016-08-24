#pragma once

#include "data/Session.h"
#include <QEvent>
#include <memory>
#include <vector>

class DetectionEvent : public QEvent {
public:
  DetectionEvent(std::unique_ptr<std::vector<std::string>> &&names,
                 std::unique_ptr<Session> &&session);

  std::unique_ptr<std::vector<std::string>> takeNames();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::vector<std::string>> _names;
  std::unique_ptr<Session> _session;
};
