#include "PerspectiveClient.h"
#include "SignatureMessage.pb.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include <grpc++/grpc++.h>

PerspectiveClient::PerspectiveClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::PerspectiveService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool PerspectiveClient::onSignature(const std::vector<int> &wordIds,
                                    const std::vector<cv::KeyPoint> &keyPoints,
                                    const CameraModel &camera,
                                    const std::vector<int> &signatureIds,
                                    const Session &session) {
  // Data we are sending to the server.
  proto::SignatureMessage signature;

  for (int wordId : wordIds) {
    signature.add_wordids(wordId);
  }

  for (const cv::KeyPoint &keyPoint : keyPoints) {
    proto::KeyPoint *protoKeyPoint = signature.add_keypoints();
    protoKeyPoint->set_x(keyPoint.pt.x);
    protoKeyPoint->set_y(keyPoint.pt.y);
    protoKeyPoint->set_size(keyPoint.size);
    protoKeyPoint->set_angle(keyPoint.angle);
    protoKeyPoint->set_response(keyPoint.response);
    protoKeyPoint->set_octave(keyPoint.octave);
    protoKeyPoint->set_classid(keyPoint.class_id);
  }

  signature.mutable_cameramodel()->set_name(camera.name());
  signature.mutable_cameramodel()->set_fx(camera.fx());
  signature.mutable_cameramodel()->set_fy(camera.fy());
  signature.mutable_cameramodel()->set_cx(camera.cx());
  signature.mutable_cameramodel()->set_cy(camera.cy());
  signature.mutable_cameramodel()->set_width(camera.getImageSize().width);
  signature.mutable_cameramodel()->set_height(camera.getImageSize().height);

  for (int signatureId : signatureIds) {
    signature.add_signatureids(signatureId);
  }

  signature.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    signature.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    signature.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  signature.mutable_session()->set_overallstart(session.overallStart);
  signature.mutable_session()->set_overallend(session.overallEnd);
  signature.mutable_session()->set_featuresstart(session.featuresStart);
  signature.mutable_session()->set_featuresend(session.featuresEnd);
  signature.mutable_session()->set_wordsstart(session.wordsStart);
  signature.mutable_session()->set_wordsend(session.wordsEnd);
  signature.mutable_session()->set_signaturesstart(session.signaturesStart);
  signature.mutable_session()->set_signaturesend(session.signaturesEnd);
  signature.mutable_session()->set_perspectivestart(session.perspectiveStart);
  signature.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onSignature(&context, signature, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
