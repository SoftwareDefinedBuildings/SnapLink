#pragma once

#include <QObject>
#include <memory>
#include <opencv2/core/types.hpp>
#include "front_end/http/HTTPFrontEnd.h"
#include "data/CameraModel.h"
#include "data/Session.h"

#define PORT 8080
#define MAX_CLIENTS 10

class IdentificationObj;

// to keep session dependent data 
struct SessionData final {
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<std::string>> names;
  QSemaphore detected;
};

class HTTPFrontEndObj final : public QObject {
public:
  explicit HTTPFrontEndObj();
  ~HTTPFrontEndObj();

  bool init(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
  void stop();

  void setIdentificationObj(std::shared_ptr<IdentificationObj> identObj);

protected:
  virtual bool event(QEvent *event);

private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera);

private:
  std::unique_ptr<HTTPFrontEnd> _httpFront;
  std::shared_ptr<IdentificationObj> _identObj;
  std::mutex _mutex; // protect _sessionMap accesses from onQuery
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<SessionData>> _sessionMap;
};
