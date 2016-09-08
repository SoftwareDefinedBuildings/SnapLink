#include "HTTP.grpc.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include <grpc++/grpc++.h>
#include <iostream>
#include <memory>
#include <string>

class HTTPClient {
public:
  explicit HTTPClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onDetection(const std::vector<std::string> &names,
                   const Session &session);

private:
  std::unique_ptr<grpc::ClientAsyncResponseReader<proto::Empty>> _rpc;
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::HTTP::Stub> stub_;

  std::shared_ptr<grpc::Channel> _channel;
};
