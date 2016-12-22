#include "FrontService.grpc.pb.h"
#include <grpc++/grpc++.h>

class Session;

class HTTPFrontEndClient {
public:
  explicit HTTPFrontEndClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onDetection(const std::vector<std::string> &names,
                   const Session &session);

private:
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::FrontService::Stub> stub_;
};
