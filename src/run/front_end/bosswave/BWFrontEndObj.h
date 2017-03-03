#pragma once

#include <QObject>
#include <memory>
#include <mutex>
#include <opencv2/core/types.hpp>
#include "lib/front_end/bosswave/BWFrontEnd.h"
#include "lib/data/CameraModel.h"
#include "lib/data/Session.h"

class IdentificationObj;

// to keep session dependent data
struct BWSessionData final {
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<std::string>> names;
  QSemaphore detected;
};

class BWFrontEndObj final : public QObject {
public:
  explicit BWFrontEndObj(const std::string &uri);
  virtual ~BWFrontEndObj();

  bool init();
  void stop();

  void setIdentificationObj(std::shared_ptr<IdentificationObj> identObj);

protected:
  virtual bool event(QEvent *event);

private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera);

private:
  std::unique_ptr<BWFrontEnd> _bwFront;
  std::shared_ptr<IdentificationObj> _identObj;
  std::mutex _mutex; // protect _sessionMap accesses from onQuery
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<BWSessionData>> _sessionMap;
};
