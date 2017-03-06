#pragma once

#include "run/data/Session.h"
#include <QEvent>
#include <memory>
#include <vector>

class DetectionEvent final : public QEvent {
public:
  explicit DetectionEvent(std::unique_ptr<std::vector<std::string>> &&results,
                 std::unique_ptr<Session> &&session);

  std::unique_ptr<std::vector<std::string>> takeResults();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::vector<std::string>> _results;
  std::unique_ptr<Session> _session;
};
