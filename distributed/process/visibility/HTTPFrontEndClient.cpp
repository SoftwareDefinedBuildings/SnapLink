#include "HTTPFrontEndClient.h"
#include "data/Session.h"

HTTPFrontEndClient::HTTPFrontEndClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::FrontEndService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool HTTPFrontEndClient::onDetection(const std::vector<std::string> &names,
                                     const Session &session) {
  // Data we are sending to the server.
  proto::DetectionMessage detection;
  for (const std::string &name : names) {
    detection.add_names(name);
  }
  detection.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    detection.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    detection.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  detection.mutable_session()->set_overallstart(session.overallStart);
  detection.mutable_session()->set_overallend(session.overallEnd);
  detection.mutable_session()->set_featuresstart(session.featuresStart);
  detection.mutable_session()->set_featuresend(session.featuresEnd);
  detection.mutable_session()->set_wordsstart(session.wordsStart);
  detection.mutable_session()->set_wordsend(session.wordsEnd);
  detection.mutable_session()->set_perspectivestart(session.perspectiveStart);
  detection.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onDetection(&context, detection, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
