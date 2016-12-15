#pragma once

#include <QObject>
#include <memory>

#define PORT 8080
#define MAX_CLIENTS 10

class CameraModel;
class Identification;

typedef struct {
  QSemaphore detected;
  std::unique_ptr<std::vector<std::string>> names;
} SessionInfo;

class HTTPFrontEndObj : public QObject {
public:
  HTTPFrontEndObj();
  virtual ~HTTPFrontEndObj();

  bool start(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
  void stop();

  void setIdentification(std::shared_ptr<Identification> identification);

protected:
  virtual bool event(QEvent *event);


private:
  // thread safe
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera,
             std::unique_ptr<Session> &&session);

private:
  std::unique_ptr<HTTPFrontEnd> _httpFront;
  std::shared_ptr<Identification> _identification;
  std::mutex _mutex; // protect _sessionInfoMap accesses from onQuery
  std::map<long, SessionInfo *> _sessionInfoMap;
};
