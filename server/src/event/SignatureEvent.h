#pragma once

#include "data/CameraModel.h"
#include "data/Session.h"
#include <QEvent>
#include <memory>

class SignatureEvent : public QEvent {
public:
  SignatureEvent(std::unique_ptr<std::multimap<int, cv::KeyPoint>> &&words,
                 std::unique_ptr<CameraModel> &&camera,
                 std::unique_ptr<std::vector<int>> &&signatureIds,
                 std::unique_ptr<Session> &&session);

  std::unique_ptr<std::multimap<int, cv::KeyPoint>> takeWords();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::unique_ptr<std::vector<int>> takeSignatureIds();
  std::unique_ptr<Session> takeSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::multimap<int, cv::KeyPoint>> _words;
  std::unique_ptr<CameraModel> _camera;
  std::unique_ptr<std::vector<int>> _signatureIds;
  std::unique_ptr<Session> _session;
};
