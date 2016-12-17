#pragma once

#include <QObject>
#include <memory>

#define PORT 8080
#define MAX_CLIENTS 10

class CameraModel;
class IdentObj;

class HTTPFrontEndObj : public QObject {
public:
  HTTPFrontEndObj();
  virtual ~HTTPFrontEndObj();

  bool start(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
  void stop();

  void setIdentObj(std::shared_ptr<IdentObj> identObj);

protected:
  virtual bool event(QEvent *event);

private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera,
             std::unique_ptr<Session> &&session);

private:
  std::unique_ptr<HTTPFrontEnd> _httpFront;
  std::shared_ptr<IdentObj> _identObj;
  std::mutex _mutex; // protect _sessionInfoMap accesses from onQuery
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<Session>> _sessionMap;
};
