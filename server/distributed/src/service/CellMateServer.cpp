#include "service/CellMateServer.h"
#include "CellMate.grpc.pb.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "util/Time.h"
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// Take in the "service" instance (in this case representing an asynchronous
// server) and the completion queue "cq" used for asynchronous communication
// with the gRPC runtime.
CellMateServer::CallData::CallData(proto::CellMate::AsyncService *service,
                                   grpc::ServerCompletionQueue *cq,
                                   Feature &feature, WordSearch &wordSearch,
                                   SignatureSearch &signatureSearch,
                                   Perspective &perspective,
                                   Visibility &visibility)
    : service_(service), cq_(cq), responder_(&ctx_), status_(CREATE),
      _feature(feature), _wordSearch(wordSearch),
      _signatureSearch(signatureSearch), _perspective(perspective),
      _visibility(visibility) {
  // Invoke the serving logic right away.
  proceed();
}

void CellMateServer::CallData::proceed() {
  if (status_ == CREATE) {
    // Make this instance progress to the PROCESS state.
    status_ = PROCESS;

    // As part of the initial CREATE state, we *request* that the system
    // start processing SayHello requests. In this request, "this" acts are
    // the tag uniquely identifying the request (so that different CallData
    // instances can serve different requests concurrently), in this case
    // the memory address of this CallData instance.
    service_->RequestOnQuery(&ctx_, &query_, &responder_, cq_, cq_, this);
  } else if (status_ == PROCESS) {
    // Spawn a new CallData instance to serve new clients while we process
    // the one for this CallData. The instance will deallocate itself as
    // part of its FINISH state.
    new CallData(service_, cq_, _feature, _wordSearch, _signatureSearch,
                 _perspective, _visibility);

    // The actual processing.
    const bool copyData = false;
    std::vector<char> data(query_.image().begin(), query_.image().end());
    cv::Mat image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

    double fx = query_.cameramodel().fx();
    double fy = query_.cameramodel().fy();
    double cx = query_.cameramodel().cx();
    double cy = query_.cameramodel().cy();
    int width = image.cols;
    int height = image.rows;
    CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));

    Session session;
    session.id = query_.session().id();
    if (query_.session().type() == proto::Session::HTTP_POST) {
      session.type = HTTP_POST;
    } else if (query_.session().type() == proto::Session::BOSSWAVE) {
      session.type = BOSSWAVE;
    }
    session.overallStart = query_.session().overallstart();
    session.overallEnd = query_.session().overallend();
    session.featuresStart = query_.session().featuresstart();
    session.featuresEnd = query_.session().featuresend();
    session.wordsStart = query_.session().wordsstart();
    session.wordsEnd = query_.session().wordsend();
    session.signaturesStart = query_.session().signaturesstart();
    session.signaturesEnd = query_.session().signaturesend();
    session.perspectiveStart = query_.session().perspectivestart();
    session.perspectiveEnd = query_.session().perspectiveend();

    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat descriptors;
    session.featuresStart = getTime();
    _feature.extract(image, keyPoints, descriptors);
    session.featuresEnd = getTime();

    session.wordsStart = getTime();
    std::vector<int> wordIds = _wordSearch.search(descriptors);
    session.wordsEnd = getTime();

    session.signaturesStart = getTime();
    std::vector<int> signatureIds = _signatureSearch.search(wordIds);
    session.signaturesEnd = getTime();

    int dbId;
    Transform pose;
    session.perspectiveStart = getTime();
    _perspective.localize(wordIds, keyPoints, camera, signatureIds.at(0), dbId,
                          pose);
    session.perspectiveEnd = getTime();

    std::vector<std::string> names = _visibility.process(dbId, camera, pose);

    // And we are done! Let the gRPC runtime know we've finished, using the
    // memory address of this instance as the uniquely identifying tag for
    // the event.
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    // Once in the FINISH state, deallocate ourselves (CallData).
    delete this;
  }
}

CellMateServer::~CellMateServer() {
  server_->Shutdown();
  // Always shutdown the completion queue after the server.
  cq_->Shutdown();
}

bool CellMateServer::init(std::vector<std::string> dbfiles) {
  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _feature.reset(new Feature());
  _wordSearch.reset(new WordSearch(std::move(words)));
  _signatureSearch.reset(new SignatureSearch(signatures));
  _perspective.reset(new Perspective(signatures));
  _visibility.reset(new Visibility(std::move(labels)));

  return true;
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
  new CallData(&service_, cq_.get(), *_feature, *_wordSearch, *_signatureSearch,
               *_perspective, *_visibility);
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
