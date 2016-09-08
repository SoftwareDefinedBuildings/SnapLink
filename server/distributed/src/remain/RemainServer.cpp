#include "remain/RemainServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "remain/HTTPClient.h"
#include "util/Time.h"
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

bool RemainServer::init(std::vector<std::string> dbfiles) {
  _channel = grpc::CreateChannel("localhost:50053",
                                 grpc::InsecureChannelCredentials());

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _wordSearch.reset(new WordSearch(std::move(words)));
  _signatureSearch.reset(new SignatureSearch(signatures));
  _perspective.reset(new Perspective(signatures));
  _visibility.reset(new Visibility(std::move(labels)));

  return true;
}

grpc::Status RemainServer::onFeature(grpc::ServerContext *context,
                                     const proto::FeatureMessage *request,
                                     proto::Empty *response) {
  //  const bool copyData = false;
  //  std::vector<char> data(request->image().begin(), request->image().end());
  //  cv::Mat image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);
  //
  //  double fx = request->cameramodel().fx();
  //  double fy = request->cameramodel().fy();
  //  double cx = request->cameramodel().cx();
  //  double cy = request->cameramodel().cy();
  //  int width = image.cols;
  //  int height = image.rows;
  //  CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));
  //
  //  Session session;
  //  session.id = request->session().id();
  //  if (request->session().type() == proto::Session::HTTP_POST) {
  //    session.type = HTTP_POST;
  //  } else if (request->session().type() == proto::Session::BOSSWAVE) {
  //    session.type = BOSSWAVE;
  //  }
  //  session.overallStart = request->session().overallstart();
  //  session.overallEnd = request->session().overallend();
  //  session.featuresStart = request->session().featuresstart();
  //  session.featuresEnd = request->session().featuresend();
  //  session.wordsStart = request->session().wordsstart();
  //  session.wordsEnd = request->session().wordsend();
  //  session.signaturesStart = request->session().signaturesstart();
  //  session.signaturesEnd = request->session().signaturesend();
  //  session.perspectiveStart = request->session().perspectivestart();
  //  session.perspectiveEnd = request->session().perspectiveend();
  //
  //  std::vector<cv::KeyPoint> keyPoints;
  //  cv::Mat descriptors;
  //  session.featuresStart = getTime();
  //  _feature->extract(image, keyPoints, descriptors);
  //  session.featuresEnd = getTime();
  //
  //  session.wordsStart = getTime();
  //  std::vector<int> wordIds = _wordSearch->search(descriptors);
  //  session.wordsEnd = getTime();
  //
  //  session.signaturesStart = getTime();
  //  std::vector<int> signatureIds = _signatureSearch->search(wordIds);
  //  session.signaturesEnd = getTime();
  //
  //  int dbId;
  //  Transform pose;
  //  session.perspectiveStart = getTime();
  //  _perspective->localize(wordIds, keyPoints, camera, signatureIds.at(0),
  //  dbId,
  //                         pose);
  //  session.perspectiveEnd = getTime();
  //
  //  std::vector<std::string> names = _visibility->process(dbId, camera, pose);
  //
  std::vector<std::string> names(1, "test");
  Session session;
  HTTPClient client(_channel);
  client.onDetection(names, session);

  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void RemainServer::run() {
  std::string server_address("0.0.0.0:50052");

  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(this);
  // Finally assemble the server.
  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}
