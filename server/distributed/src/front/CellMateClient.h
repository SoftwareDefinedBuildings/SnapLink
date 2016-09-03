#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

#include "CellMate.grpc.pb.h"

class CellMateClient {
public:
  explicit CellMateClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  detect(const std::vector<char> &image, const CameraModel &camera,
         const Session &session);

  void finish(Query *reply, grpc::Status *status, void *tag);

private:
  std::unique_ptr<ClientAsyncResponseReader<Query>> _rpc;
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<CellMate::Stub> stub_;

  std::shared_ptr<grpc::Channel> _channel;
};
