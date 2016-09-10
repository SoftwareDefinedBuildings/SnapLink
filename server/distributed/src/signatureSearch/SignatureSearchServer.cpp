#include "signatureSearch/SignatureSearchServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "signatureSearch/RemainClient.h"
#include "util/Time.h"
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

bool SignatureSearchServer::init(std::vector<std::string> dbfiles) {
  _channel = grpc::CreateChannel("localhost:50054",
                                 grpc::InsecureChannelCredentials());

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _signatureSearch.reset(new SignatureSearch(signatures));

  return true;
}

grpc::Status SignatureSearchServer::onWord(grpc::ServerContext *context,
                                           const proto::WordMessage *request,
                                           proto::Empty *response) {
  std::vector<int> wordIds;
  for (int i = 0; i < request->wordids_size(); i++) {
    wordIds.emplace_back(request->wordids(i));
  }

  std::vector<cv::KeyPoint> keyPoints;
  for (int i = 0; i < request->keypoints_size(); i++) {
    float x = request->keypoints(i).x();
    float y = request->keypoints(i).y();
    float size = request->keypoints(i).size();
    float angle = request->keypoints(i).angle();
    float response = request->keypoints(i).response();
    int octave = request->keypoints(i).octave();
    int classId = request->keypoints(i).classid();

    keyPoints.emplace_back(x, y, size, angle, response, octave, classId);
  }

  double fx = request->cameramodel().fx();
  double fy = request->cameramodel().fy();
  double cx = request->cameramodel().cx();
  double cy = request->cameramodel().cy();
  int width = request->cameramodel().width();
  int height = request->cameramodel().height();
  CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));

  Session session;
  session.id = request->session().id();
  if (request->session().type() == proto::Session::HTTP_POST) {
    session.type = HTTP_POST;
  } else if (request->session().type() == proto::Session::BOSSWAVE) {
    session.type = BOSSWAVE;
  }
  session.overallStart = request->session().overallstart();
  session.overallEnd = request->session().overallend();
  session.featuresStart = request->session().featuresstart();
  session.featuresEnd = request->session().featuresend();
  session.wordsStart = request->session().wordsstart();
  session.wordsEnd = request->session().wordsend();
  session.signaturesStart = request->session().signaturesstart();
  session.signaturesEnd = request->session().signaturesend();
  session.perspectiveStart = request->session().perspectivestart();
  session.perspectiveEnd = request->session().perspectiveend();

  session.signaturesStart = getTime();
  std::vector<int> signatureIds = _signatureSearch->search(wordIds);
  session.signaturesEnd = getTime();

  RemainClient client(_channel);
  client.onSignature(wordIds, keyPoints, camera, signatureIds, session);

  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void SignatureSearchServer::run() {
  std::string server_address("0.0.0.0:50053");

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
