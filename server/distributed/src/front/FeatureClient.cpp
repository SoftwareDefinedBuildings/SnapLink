#include "front/FeatureClient.h"
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

FeatureClient::FeatureClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::FeatureService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool FeatureClient::onQuery(const std::vector<char> &image,
                            const CameraModel &camera, const Session &session) {
  // Data we are sending to the server.
  proto::QueryMessage query;

  query.set_image(std::string(image.begin(), image.end()));

  query.mutable_cameramodel()->set_name(camera.name());
  query.mutable_cameramodel()->set_fx(camera.fx());
  query.mutable_cameramodel()->set_fy(camera.fy());
  query.mutable_cameramodel()->set_cx(camera.cx());
  query.mutable_cameramodel()->set_cy(camera.cy());

  query.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    query.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    query.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  query.mutable_session()->set_overallstart(session.overallStart);
  query.mutable_session()->set_overallend(session.overallEnd);
  query.mutable_session()->set_featuresstart(session.featuresStart);
  query.mutable_session()->set_featuresend(session.featuresEnd);
  query.mutable_session()->set_wordsstart(session.wordsStart);
  query.mutable_session()->set_wordsend(session.wordsEnd);
  query.mutable_session()->set_signaturesstart(session.signaturesStart);
  query.mutable_session()->set_signaturesend(session.signaturesEnd);
  query.mutable_session()->set_perspectivestart(session.perspectiveStart);
  query.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onQuery(&context, query, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
