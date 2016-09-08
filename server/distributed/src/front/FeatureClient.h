#include "FeatureService.grpc.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include <grpc++/grpc++.h>
#include <iostream>
#include <memory>
#include <string>

class FeatureClient {
public:
  FeatureClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onQuery(const std::vector<char> &image, const CameraModel &camera,
               const Session &session);

private:
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::FeatureService::Stub> stub_;
};
