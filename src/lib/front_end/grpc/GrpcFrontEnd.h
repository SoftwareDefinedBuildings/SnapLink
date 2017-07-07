#pragma once

#include <QObject>
#include <QThread>
#include "lib/front_end/FrontEnd.h"
#include <atomic>
#include <mutex>  
#include "GrpcService.grpc.pb.h"
#include <grpc++/grpc++.h>

class FoundItem;
class GrpcFrontEnd final : public QObject, public cellmate_grpc::GrpcService::Service, public FrontEnd{
  Q_OBJECT

public:
  explicit GrpcFrontEnd(int grpcServerAddr, unsigned int maxClients);
  ~GrpcFrontEnd();

  bool start() final;
  void stop() final;
  grpc::Status onClientQuery(grpc::ServerContext *context,
                            grpc::ServerReaderWriter<cellmate_grpc::ServerRespondMessage, cellmate_grpc::ClientQueryMessage> *stream);
public slots:
  void run();

private:
  cv::Mat rotateClockwise(cv::Mat src, double angle);
  void rotateBack(std::vector<FoundItem> &results ,double angle, int width, int height);
private:
  static const std::string none;
  QThread _thread;
  std::string _serverAddress;
  std::atomic<unsigned int> _numClients;
  std::atomic<unsigned int> _maxClients; 
  std::mutex _mutex;
};
