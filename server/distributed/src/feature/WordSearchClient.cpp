#include "feature/WordSearchClient.h"
#include <cassert>
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

WordSearchClient::WordSearchClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::WordSearchService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool WordSearchClient::onFeature(const std::vector<cv::KeyPoint> &keyPoints,
                             const cv::Mat &descriptors,
                             const CameraModel &camera,
                             const Session &session) {
  // Data we are sending to the server.
  proto::FeatureMessage feature;

  for (const cv::KeyPoint &keyPoint : keyPoints) {
    proto::KeyPoint *protoKeyPoint = feature.add_keypoints();
    protoKeyPoint->set_x(keyPoint.pt.x);
    protoKeyPoint->set_y(keyPoint.pt.y);
    protoKeyPoint->set_size(keyPoint.size);
    protoKeyPoint->set_angle(keyPoint.angle);
    protoKeyPoint->set_response(keyPoint.response);
    protoKeyPoint->set_octave(keyPoint.octave);
    protoKeyPoint->set_classid(keyPoint.class_id);
  }

  assert(descriptors.type() == CV_32F);
  assert(descriptors.channels() == 1);
  for (int row = 0; row < descriptors.rows; row++) {
    const float *p = descriptors.ptr<float>(row);
    proto::Descriptor *descriptor = feature.add_descriptors();
    for (int col = 0; col < descriptors.cols; col++) {
      descriptor->add_values(p[col]);
    }
  }

  feature.mutable_cameramodel()->set_name(camera.name());
  feature.mutable_cameramodel()->set_fx(camera.fx());
  feature.mutable_cameramodel()->set_fy(camera.fy());
  feature.mutable_cameramodel()->set_cx(camera.cx());
  feature.mutable_cameramodel()->set_cy(camera.cy());
  feature.mutable_cameramodel()->set_width(camera.getImageSize().width);
  feature.mutable_cameramodel()->set_height(camera.getImageSize().height);

  feature.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    feature.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    feature.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  feature.mutable_session()->set_overallstart(session.overallStart);
  feature.mutable_session()->set_overallend(session.overallEnd);
  feature.mutable_session()->set_featuresstart(session.featuresStart);
  feature.mutable_session()->set_featuresend(session.featuresEnd);
  feature.mutable_session()->set_wordsstart(session.wordsStart);
  feature.mutable_session()->set_wordsend(session.wordsEnd);
  feature.mutable_session()->set_signaturesstart(session.signaturesStart);
  feature.mutable_session()->set_signaturesend(session.signaturesEnd);
  feature.mutable_session()->set_perspectivestart(session.perspectiveStart);
  feature.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onFeature(&context, feature, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
