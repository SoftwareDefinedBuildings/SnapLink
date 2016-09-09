#include "wordSearch/RemainClient.h"
#include "WordMessage.pb.h"
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

RemainClient::RemainClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::RemainService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool RemainClient::onWord(const std::vector<int> &wordIds,
                   const std::vector<cv::KeyPoint> &keyPoints,
                   const CameraModel &camera,
                   const Session &session) {
  // Data we are sending to the server.
  proto::WordMessage word;

  for (int wordId : wordIds) {
    word.add_wordids(wordId);
  }

  for (const cv::KeyPoint &keyPoint : keyPoints) {
    proto::KeyPoint *protoKeyPoint = word.add_keypoints();
    protoKeyPoint->set_x(keyPoint.pt.x);
    protoKeyPoint->set_y(keyPoint.pt.y);
    protoKeyPoint->set_size(keyPoint.size);
    protoKeyPoint->set_angle(keyPoint.angle);
    protoKeyPoint->set_response(keyPoint.response);
    protoKeyPoint->set_octave(keyPoint.octave);
    protoKeyPoint->set_classid(keyPoint.class_id);
  }

  word.mutable_cameramodel()->set_name(camera.name());
  word.mutable_cameramodel()->set_fx(camera.fx());
  word.mutable_cameramodel()->set_fy(camera.fy());
  word.mutable_cameramodel()->set_cx(camera.cx());
  word.mutable_cameramodel()->set_cy(camera.cy());
  word.mutable_cameramodel()->set_width(camera.getImageSize().width);
  word.mutable_cameramodel()->set_height(camera.getImageSize().height);

  word.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    word.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    word.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  word.mutable_session()->set_overallstart(session.overallStart);
  word.mutable_session()->set_overallend(session.overallEnd);
  word.mutable_session()->set_featuresstart(session.featuresStart);
  word.mutable_session()->set_featuresend(session.featuresEnd);
  word.mutable_session()->set_wordsstart(session.wordsStart);
  word.mutable_session()->set_wordsend(session.wordsEnd);
  word.mutable_session()->set_signaturesstart(session.signaturesStart);
  word.mutable_session()->set_signaturesend(session.signaturesEnd);
  word.mutable_session()->set_perspectivestart(session.perspectiveStart);
  word.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onWord(&context, word, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
