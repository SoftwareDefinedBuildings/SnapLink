#pragma once

#include "data/CameraModel.h"
#include "data/PerfData.h"
#include "data/Signature.h"
#include <QEvent>

class SignatureEvent : public QEvent {
public:
  SignatureEvent(std::unique_ptr<std::multimap<int, cv::KeyPoint>> &&words,
                 std::unique_ptr<CameraModel> &&camera,
                 std::vector<std::unique_ptr<Signature>> &&signatures,
                 std::unique_ptr<PerfData> &&perfData, const void *session);

  std::unique_ptr<std::multimap<int, cv::KeyPoint>> takeWords();
  std::unique_ptr<CameraModel> takeCameraModel();
  std::vector<std::unique_ptr<Signature>> takeSignatures();
  std::unique_ptr<PerfData> takePerfData();
  const void *getSession();

  static QEvent::Type type();

private:
  static const QEvent::Type _type;
  std::unique_ptr<std::multimap<int, cv::KeyPoint>> _words;
  std::unique_ptr<CameraModel> _camera;
  std::vector<std::unique_ptr<Signature>> _signatures;
  std::unique_ptr<PerfData> _perfData;
  const void *_session;
};
