#include "FeatureService.grpc.pb.h"
#include <grpc++/grpc++.h>
#include <opencv2/core/types.hpp>
#include <memory>
#include <string>

class CameraModel;
class Session;

class FeatureClient final {
public:
  FeatureClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onQuery(const cv::Mat &image, const CameraModel &camera,
               const Session &session);

private:
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::FeatureService::Stub> stub_;
};
