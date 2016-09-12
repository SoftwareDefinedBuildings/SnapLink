#include "perspective/PerspectiveServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "perspective/VisibilityClient.h"
#include "util/Time.h"
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

bool PerspectiveServer::init(std::vector<std::string> dbfiles) {
  _channel = grpc::CreateChannel("localhost:50055",
                                 grpc::InsecureChannelCredentials());

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _perspective.reset(new Perspective(signatures));

  return true;
}

grpc::Status
PerspectiveServer::onSignature(grpc::ServerContext *context,
                               const proto::SignatureMessage *request,
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

  std::vector<int> signatureIds;
  for (int i = 0; i < request->signatureids_size(); i++) {
    signatureIds.emplace_back(request->signatureids(i));
  }

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

  int dbId;
  Transform pose;
  session.perspectiveStart = getTime();
  _perspective->localize(wordIds, keyPoints, camera, signatureIds.at(0), dbId,
                         pose);
  session.perspectiveEnd = getTime();

  VisibilityClient client(_channel);
  client.onLocation(dbId, camera, pose, session);

  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void PerspectiveServer::run() {
  std::string server_address("0.0.0.0:50054");

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
