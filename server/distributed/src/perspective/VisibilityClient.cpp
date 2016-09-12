#include "VisibilityClient.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "data/Transform.h"
#include <iostream>
#include <memory>

VisibilityClient::VisibilityClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::VisibilityService::NewStub(channel)) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
bool VisibilityClient::onLocation(int dbId, const CameraModel &camera,
                                  const Transform &pose,
                                  const Session &session) {
  // Data we are sending to the server.
  proto::LocationMessage location;

  location.set_dbid(dbId);

  location.mutable_cameramodel()->set_name(camera.name());
  location.mutable_cameramodel()->set_fx(camera.fx());
  location.mutable_cameramodel()->set_fy(camera.fy());
  location.mutable_cameramodel()->set_cx(camera.cx());
  location.mutable_cameramodel()->set_cy(camera.cy());
  location.mutable_cameramodel()->set_width(camera.getImageSize().width);
  location.mutable_cameramodel()->set_height(camera.getImageSize().height);

  location.mutable_pose()->set_r11(pose.r11());
  location.mutable_pose()->set_r12(pose.r12());
  location.mutable_pose()->set_r13(pose.r13());
  location.mutable_pose()->set_r21(pose.r21());
  location.mutable_pose()->set_r22(pose.r22());
  location.mutable_pose()->set_r23(pose.r23());
  location.mutable_pose()->set_r31(pose.r31());
  location.mutable_pose()->set_r32(pose.r32());
  location.mutable_pose()->set_r33(pose.r33());
  location.mutable_pose()->set_x(pose.x());
  location.mutable_pose()->set_y(pose.y());
  location.mutable_pose()->set_z(pose.z());

  location.mutable_session()->set_id(session.id);
  if (session.type == HTTP_POST) {
    location.mutable_session()->set_type(proto::Session::HTTP_POST);
  } else if (session.type == BOSSWAVE) {
    location.mutable_session()->set_type(proto::Session::BOSSWAVE);
  }
  location.mutable_session()->set_overallstart(session.overallStart);
  location.mutable_session()->set_overallend(session.overallEnd);
  location.mutable_session()->set_featuresstart(session.featuresStart);
  location.mutable_session()->set_featuresend(session.featuresEnd);
  location.mutable_session()->set_wordsstart(session.wordsStart);
  location.mutable_session()->set_wordsend(session.wordsEnd);
  location.mutable_session()->set_signaturesstart(session.signaturesStart);
  location.mutable_session()->set_signaturesend(session.signaturesEnd);
  location.mutable_session()->set_perspectivestart(session.perspectiveStart);
  location.mutable_session()->set_perspectiveend(session.perspectiveEnd);

  // Container for the data we expect from the server.
  proto::Empty reply;

  // Context for the client. It could be used to convey extra information to
  // the server and/or tweak certain RPC behaviors.
  grpc::ClientContext context;

  // The actual RPC.
  grpc::Status status = stub_->onLocation(&context, location, &reply);

  // Act upon its status.
  if (status.ok()) {
    return true;
  } else {
    std::cout << status.error_code() << ": " << status.error_message()
              << std::endl;
    return false;
  }
}
