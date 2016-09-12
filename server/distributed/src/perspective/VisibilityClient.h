#include "VisibilityService.grpc.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include <grpc++/grpc++.h>
#include <iostream>
#include <memory>
#include <string>

class VisibilityClient {
public:
  explicit VisibilityClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onLocation(int dbId, const CameraModel &camera, const Transform &pose,
                  const Session &session);

private:
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::VisibilityService::Stub> stub_;
};
