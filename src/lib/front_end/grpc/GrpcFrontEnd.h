#pragma once

#include "lib/front_end/FrontEnd.h"
#include <atomic>
#include <mutex>  
#include "GrpcService.grpc.pb.h"
#include <grpc++/grpc++.h>
class GrpcFrontEnd final : public proto::GrpcFrontEndService::Service {
public:
  explicit GrpcFrontEnd(const std::string &grpcServerAddr, unsigned int maxClients);
  ~GrpcFrontEnd();

  bool start() final;
  void stop() final;



private:
  std::string _serverAddress;
  std::atomic<unsigned int> _numClients;
  std::atomic<unsigned int> _maxClients; 
  std::mutex _mutex;
}
