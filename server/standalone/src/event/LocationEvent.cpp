#include "event/LocationEvent.h"

const QEvent::Type LocationEvent::_type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

LocationEvent::LocationEvent(int dbId, std::unique_ptr<CameraModel> &&camera,
                             std::unique_ptr<Transform> &&pose,
                             std::unique_ptr<Session> &&session)
    : QEvent(LocationEvent::type()), _dbId(dbId), _camera(std::move(camera)),
      _pose(std::move(pose)), _session(std::move(session)) {}

int LocationEvent::dbId() const { return _dbId; }

std::unique_ptr<CameraModel> LocationEvent::takeCameraModel() {
  return std::move(_camera);
}

std::unique_ptr<Transform> LocationEvent::takePose() {
  return std::move(_pose);
}

std::unique_ptr<Session> LocationEvent::takeSession() {
  return std::move(_session);
}

QEvent::Type LocationEvent::type() { return _type; }
