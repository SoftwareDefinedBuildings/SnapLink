#include "FeatureMessage.pb.h"
#include "FrontService.grpc.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include <grpc++/grpc++.h>
#include <iostream>
#include <memory>
#include <string>

class RemainClient {
public:
  explicit RemainClient(std::shared_ptr<grpc::Channel> channel);

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  bool onFeature(const std::vector<cv::KeyPoint> &keyPoints,
                 const cv::Mat &descriptors, const CameraModel &camera,
                 const Session &session);

private:
  std::unique_ptr<grpc::ClientAsyncResponseReader<proto::Empty>> _rpc;
  // Out of the passed in Channel comes the stub, stored here, our view of the
  // server's exposed services.
  std::unique_ptr<proto::FrontService::Stub> stub_;

  std::shared_ptr<grpc::Channel> _channel;
};
