#include "feature/RemainClient.h"
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

RemainClient::RemainClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::FrontService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool RemainClient::onFeature(const std::vector<cv::KeyPoint> &keyPoints,
                             const cv::Mat &descriptors,
                             const CameraModel &camera,
                             const Session &session) {
  // Data we are sending to the server.
  //   proto::FeatureMessage feature;
  //   for (const cv::KeyPoint &keyPoint : keyPoints) {
  //     proto::KeyPoint
  //     .add_names(name);
  //   }
  //   feature.mutable_session()->set_id(session.id);
  //   if (session.type == HTTP_POST) {
  //     feature.mutable_session()->set_type(proto::Session::HTTP_POST);
  //   } else if (session.type == BOSSWAVE) {
  //     feature.mutable_session()->set_type(proto::Session::BOSSWAVE);
  //   }
  //   feature.mutable_session()->set_overallstart(session.overallStart);
  //   feature.mutable_session()->set_overallend(session.overallEnd);
  //   feature.mutable_session()->set_featuresstart(session.featuresStart);
  //   feature.mutable_session()->set_featuresend(session.featuresEnd);
  //   feature.mutable_session()->set_wordsstart(session.wordsStart);
  //   feature.mutable_session()->set_wordsend(session.wordsEnd);
  //   feature.mutable_session()->set_signaturesstart(session.signaturesStart);
  //   feature.mutable_session()->set_signaturesend(session.signaturesEnd);
  //   feature.mutable_session()->set_perspectivestart(session.perspectiveStart);
  //   feature.mutable_session()->set_perspectiveend(session.perspectiveEnd);
  //
  //   // Container for the data we expect from the server.
  //   proto::Empty reply;
  //
  //   // Context for the client. It could be used to convey extra information
  //   to
  //   // the server and/or tweak certain RPC behaviors.
  //   grpc::ClientContext context;
  //
  //   // The actual RPC.
  //   grpc::Status status = stub_->onFeature(&context, feature, &reply);
  //
  //  // Act upon its status.
  //  if (status.ok()) {
  return true;
  //  } else {
  //    std::cout << status.error_code() << ": " << status.error_message()
  //              << std::endl;
  //    return false;
  //  }
}
