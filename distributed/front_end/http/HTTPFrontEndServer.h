#pragma once

#include "FrontService.grpc.pb.h"
#include "data/Session.h"
#include <QSemaphore>
#include <grpc++/grpc++.h>
#include <memory>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <random>

#define PORT 8080
#define MAX_CLIENTS 10
#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000

class HTTPFrontEndServer final : public proto::FrontService::Service {
public:
  bool init(std::string featureServerAddr, uint16_t port = PORT,
            unsigned int maxClients = MAX_CLIENTS);
  // TODO add on failure
  grpc::Status onDetection(grpc::ServerContext *context,
                           const proto::DetectionMessage *request,
                           proto::Empty *response) override;
  void run(std::string httpFrontEndServerAddr);

private:
  // this method must be thread safe, because it will be called from other threads
  std::vector<std::string> onQuery(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera);

private:
  std::unique_ptr<HTTPFrontEnd> _httpFront;
  std::mutex _mutex;
  std::mt19937 _gen;
  std::uniform_int_distribution<long> _dis;
  std::map<long, std::unique_ptr<Session>> _sessionMap;

  std::shared_ptr<grpc::Channel> _channel;
};
