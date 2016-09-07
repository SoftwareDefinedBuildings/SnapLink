#include "CellMate.grpc.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include <grpc++/grpc++.h>
#include <iostream>
#include <memory>
#include <string>

class CellMateClient {
public:
  explicit CellMateClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  void detect(const std::vector<char> &image, const CameraModel &camera,
              const Session &session);

  void finish(proto::Empty *reply, grpc::Status *status, void *tag);

private:
  std::unique_ptr<grpc::ClientAsyncResponseReader<proto::Empty>> _rpc;
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::CellMate::Stub> stub_;

  std::shared_ptr<grpc::Channel> _channel;
};
