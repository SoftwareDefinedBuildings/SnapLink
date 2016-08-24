#pragma once

#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include <QEvent>
#include <memory>

class LocationEvent : public QEvent {
public:
  LocationEvent(int dbId, std::unique_ptr<CameraModel> &&camera,
                std::unique_ptr<Transform> &&pose,
                std::unique_ptr<Session> &&session);

  int dbId() const;
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<Transform> takePose();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  int _dbId;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<Transform> _pose;
  std::unique_ptr<Session> _session;
};
