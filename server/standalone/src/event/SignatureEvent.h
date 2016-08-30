#pragma once

#include "data/CameraModel.h"
#include "data/Session.h"
#include <QEvent>
#include <memory>

class SignatureEvent : public QEvent {
public:
  SignatureEvent(std::unique_ptr<std::vector<int>> &&wordIds,
                 std::unique_ptr<std::vector<cv::KeyPoint>> &&keyPoints,
                 std::unique_ptr<CameraModel> &&camera,
                 std::unique_ptr<std::vector<int>> &&signatureIds,
                 std::unique_ptr<Session> &&session);

  std::unique_ptr<std::vector<int>> takeWordIds();
  std::unique_ptr<std::vector<cv::KeyPoint>> takeKeyPoints();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<std::vector<int>> takeSignatureIds();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::vector<int>> _wordIds;
  std::unique_ptr<std::vector<cv::KeyPoint>> _keyPoints;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<std::vector<int>> _signatureIds;
  std::unique_ptr<Session> _session;
};
