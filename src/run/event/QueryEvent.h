#pragma once

#include "lib/data/CameraModel.h"
#include "run/data/Session.h"
#include <QEvent>
#include <memory>

class QueryEvent final : public QEvent {
public:
  explicit QueryEvent(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera,
             std::unique_ptr<Session> &&session);

  std::unique_ptr<cv::Mat> takeImage();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<cv::Mat> _image;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<Session> _session;
};
