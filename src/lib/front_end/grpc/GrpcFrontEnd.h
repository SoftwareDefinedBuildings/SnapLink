#pragma once

#include <QObject>
#include <QThread>
#include "lib/front_end/FrontEnd.h"
#include <atomic>
#include <mutex>  
#include "GrpcService.grpc.pb.h"
#include <grpc++/grpc++.h>

class FoundItem;
class GrpcFrontEnd final : public QObject, public snaplink_grpc::GrpcService::Service, public FrontEnd {
  Q_OBJECT

public:
  explicit GrpcFrontEnd(int grpcServerAddr, unsigned int maxClients);
  ~GrpcFrontEnd();

  bool start() final;
  void stop() final;
  grpc::Status localize(grpc::ServerContext *context,
                             grpc::ServerReaderWriter<snaplink_grpc::LocalizationResponse, snaplink_grpc::LocalizationRequest> *stream);
  
  grpc::Status getLabels(
    grpc::ServerContext *context,
    const snaplink_grpc::Empty *empty,
    snaplink_grpc::GetLabelsResponse *response); 
public slots:
  void run();

private:
  cv::Mat rotateImage(cv::Mat src, int orientation); // orinentation is EXIF orientation
  void updateIntrinsics(int width, int height, int orientation, float &cx, float &cy);

private:
  static const std::string none;
  QThread _thread;
  std::string _serverAddress;
  std::atomic<unsigned int> _numClients;
  std::atomic<unsigned int> _maxClients; 
  std::mutex _mutex;
};
