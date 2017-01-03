#pragma once

#include <QObject>
#include <memory>
#include <opencv2/core/types.hpp>
#include "front_end/bosswave/BWFrontEnd.h"
#include "data/CameraModel.h"
#include "data/Session.h"

#define PORT 8080
#define MAX_CLIENTS 10

class IdentificationObj;

// to keep session dependent data
struct BWSessionData final {
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<std::string>> names;
  QSemaphore detected;
};

class BWFrontEndObj final : public QObject {
  Q_OBJECT
public:
  BWFrontEndObj();
  virtual ~BWFrontEndObj();

  void stop();

  void setIdentificationObj(std::shared_ptr<IdentificationObj> identObj);

public slots:
  bool init( unsigned int maxClients = MAX_CLIENTS);
signals:
  void triggerInit(unsigned int maxClients);
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
