#pragma once

#include "data/PerfData.h"
#include "data/CameraModel.h"
#include <QEvent>
#include <memory>

class FeatureEvent : public QEvent {
public:
  FeatureEvent(std::unique_ptr<std::vector<cv::KeyPoint>> &&keyPoints,
               std::unique_ptr<cv::Mat> &&descriptors,
               std::unique_ptr<CameraModel> &&camera,
               std::unique_ptr<PerfData> &&perfData, const void *session);

  std::unique_ptr<std::vector<cv::KeyPoint>> takeKeyPoints();
  std::unique_ptr<cv::Mat> takeDescriptors();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<PerfData> takePerfData();
  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::vector<cv::KeyPoint>> _keyPoints;
  std::unique_ptr<cv::Mat> _descriptors;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<PerfData> _perfData;
  const void *_session;
};
