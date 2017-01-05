#pragma once

#include "FrontEndService.grpc.pb.h"
#include "data/Session.h"
#include "data/CameraModel.h"
#include "front_end/bosswave/BWFrontEnd.h"
#include <QSemaphore>
#include <grpc++/grpc++.h>
#include <memory>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <random>

#define PORT 8080
#define MAX_CLIENTS 10

// to keep session dependent data 
typedef struct {
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<std::string>> names;
  QSemaphore detected;
} SessionData;

class BWFrontEndServer: public QObject,  public proto::FrontEndService::Service {
  Q_OBJECT
public:
  bool init(std::string featureServerAddr, 
            unsigned int maxClients = MAX_CLIENTS);
  // TODO add on failure
  grpc::Status onDetection(grpc::ServerContext *context,
                           const proto::DetectionMessage *request,
                           proto::Empty *response) override;
public slots:
  void run(QString bwFrontEndServerAddr);
signals:
  void triggerRun(QString bwFrontEndServerAddr);
private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera);

private:
  std::unique_ptr<BWFrontEnd> _bwFront;
  std::mutex _mutex;
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<SessionData>> _sessionMap;

  std::shared_ptr<grpc::Channel> _channel;
};
