#include "http/CellMateClient.h"
#include <iostream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>

CellMateClient::CellMateClient(std::shared_ptr<grpc::Channel> channel)
    : stub_(proto::CellMate::NewStub(channel)),
      _channel(grpc::CreateChannel("localhost:50051",
                                   grpc::InsecureChannelCredentials())) {}

// Assembles the client's payload, sends it and presents the response back
// from the server.
void CellMateClient::query(const std::vector<char> &image,
                           const CameraModel &camera, const Session &session) {
  // Data we are sending to the server.
  proto::Query query;
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

  // The producer-consumer queue we use to communicate asynchronously with the
  // gRPC runtime.
  grpc::CompletionQueue cq;

  // Storage for the status of the RPC upon completion.
  grpc::Status status;

  // stub_->AsyncSayHello() performs the RPC call, returning an instance we
  // store in "rpc". Because we are using the asynchronous API, we need to
  // hold on to the "rpc" instance in order to get updates on the ongoing RPC.
  _rpc = stub_->AsyncOnQuery(&context, query, &cq);

  void *got_tag;
  bool ok = false;
  // Block until the next result is available in the completion queue "cq".
  // The return value of Next should always be checked. This return value
  // tells us whether there is any kind of event or the cq_ is shutting down.
  GPR_ASSERT(cq.Next(&got_tag, &ok));

  // Verify that the result from "cq" corresponds, by its tag, our previous
  // request.
  GPR_ASSERT(got_tag == (void *)1);
  // ... and that the request was completed successfully. Note that "ok"
  // corresponds solely to the request for updates introduced by Finish().
  GPR_ASSERT(ok);
}

void CellMateClient::finish(proto::Empty *reply, grpc::Status *status,
                            void *tag) {
  // Request that, upon completion of the RPC, "reply" be updated with the
  // server's response; "status" with the indication of whether the operation
  // was successful. Tag the request with the integer 1.
  _rpc->Finish(reply, status, tag);
}
