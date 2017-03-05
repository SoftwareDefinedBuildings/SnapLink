#pragma once

#include <QObject>
#include <QEvent>
#include <QSemaphore>
#include <memory>
#include <mutex>
#include <opencv2/core/types.hpp>
#include "run/wrapper/BackEndWrapper.h"
#include "lib/front_end/FrontEnd.h"
#include "lib/data/CameraModel.h"
#include "run/data/Session.h"


// to keep session dependent data 
struct SessionData final {
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<std::string>> names;
  QSemaphore detected;
};

class FrontEndWrapper final : public QObject, public std::enable_shared_from_this<FrontEndWrapper> {
public:
  // owmership transfer
  explicit FrontEndWrapper(std::unique_ptr<FrontEnd> &&frontEnd);
  ~FrontEndWrapper();

  bool init();
  void stop();

  void setBackEndWrapper(const std::shared_ptr<BackEndWrapper> &backEndWrapper);

protected:
  bool event(QEvent *event);

private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera);

private:
  std::unique_ptr<FrontEnd> _frontEnd;
  std::shared_ptr<BackEndWrapper> _backEndWrapper;
  std::mutex _mutex; // protect _sessionMap accesses from onQuery
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<SessionData>> _sessionMap;
};
