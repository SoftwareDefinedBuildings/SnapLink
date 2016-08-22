#pragma once

#include "data/PerfData.h"
#include "data/SensorData.h"
#include <QEvent>
#include <memory>

class QueryEvent : public QEvent {
public:
  QueryEvent(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera,
             std::unique_ptr<PerfData> &&perfData, const void *session);

  std::unique_ptr<cv::Mat> takeImage();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<PerfData> takePerfData();
  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<cv::Mat> _image;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<PerfData> _perfData;
  const void *_session;
};
