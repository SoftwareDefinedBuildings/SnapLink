#include <grpc++/grpc++.h>
#include "algo/Feature.h"
#include "service/CellMate.grpc.pb.h"

// Take in the "service" instance (in this case representing an asynchronous
// server) and the completion queue "cq" used for asynchronous communication
// with the gRPC runtime.
CallData::CallData(CellMate::AsyncService *service, grpc::ServerCompletionQueue *cq, Feature &feature)
    : service_(service), cq_(cq), responder_(&ctx_), status_(CREATE), _feature(feature) {
  // Invoke the serving logic right away.
  proceed();
}

void CallData::proceed() {
  if (status_ == CREATE) {
    // Make this instance progress to the PROCESS state.
    status_ = PROCESS;

    // As part of the initial CREATE state, we *request* that the system
    // start processing SayHello requests. In this request, "this" acts are
    // the tag uniquely identifying the request (so that different CallData
    // instances can serve different requests concurrently), in this case
    // the memory address of this CallData instance.
    service_->RequestDetect(&ctx_, &request_, &responder_, cq_, cq_, this);
  } else if (status_ == PROCESS) {
    // Spawn a new CallData instance to serve new clients while we process
    // the one for this CallData. The instance will deallocate itself as
    // part of its FINISH state.
    new CallData(service_, cq_);

    // The actual processing.
    const bool copyData = false;
    std::vector<char> data(request_.image().begin(), request_.image().end());
    image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

    double fx = request_.cameramodel()->fx();
    double fy = request_.cameramodel()->fy();
    double cx = request_.cameramodel()->cx();
    double cy = request_.cameramodel()->cy();
    int width = image.cols;
    int height = image.rows;
    camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));

    Session session;
    session.id = request_.session()->id;
    session.type = request_.session()->type;
    session.overallStart = request_.session()->overallStart;
    session.overallEnd = request_.session()->overallEnd;
    session.featuresStart = request_.session()->featureStart;
    session.featuresEnd = request_.session()->featureEnd;
    session.wordsStart = request_.session()->wordsStart;
    session.wordsEnd = request_.session()->wordsEnd;
    session.signaturesStart = request_.session()->signaturesStart;
    session.signaturesEnd = request_.session()->signaturesEnd;
    session.perspectiveStart = request_.session()->perspectiveStart;
    session.perspectiveEnd = request_.session()->perspectiveEnd;

    std::unique_ptr<cv::Mat> image(new cv::Mat());
    std::unique_ptr<CameraModel> camera(new CameraModel());

    _feature.extract(image, camera, session, *image, *camera);

    // And we are done! Let the gRPC runtime know we've finished, using the
    // memory address of this instance as the uniquely identifying tag for
    // the event.
    status_ = FINISH;
    responder_.Finish(reply_, Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    // Once in the FINISH state, deallocate ourselves (CallData).
    delete this;
  }
}

~CellMateServer::CellMateServer() {
  server_->Shutdown();
  // Always shutdown the completion queue after the server.
  cq_->Shutdown();
}

// There is no shutdown handling in this code.
void CellMateServer::run() {
  std::string server_address("0.0.0.0:50051");

  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service_" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *asynchronous* service.
  builder.RegisterService(&service_);
  // Get hold of the completion queue used for the asynchronous communication
  // with the gRPC runtime.
  cq_ = builder.AddCompletionQueue();
  // Finally assemble the server.
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  // Proceed to the server's main loop.
  handleRpcs();
}

// This can be run in multiple threads if needed.
void CellMateServer::handleRpcs() {
  // Spawn a new CallData instance to serve new clients.
  new CallData(&service_, cq_.get(), _feature);
  void *tag; // uniquely identifies a request.
  bool ok;
  while (true) {
    // Block waiting to read the next event from the completion queue. The
    // event is uniquely identified by its tag, which in this case is the
    // memory address of a CallData instance.
    // The return value of Next should always be checked. This return value
    // tells us whether there is any kind of event or cq_ is shutting down.
    GPR_ASSERT(cq_->Next(&tag, &ok));
    GPR_ASSERT(ok);
    static_cast<CallData *>(tag)->proceed();
  }
}
