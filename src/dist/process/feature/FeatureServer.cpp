#include "FeatureServer.h"
#include "WordSearchClient.h"
#include "data/CameraModel.h"
#include "data/Session.h"
#include "util/Utility.h"
#include <opencv2/core/core.hpp>

bool FeatureServer::init(std::string wordSearchServerAddr) {
  _channel = grpc::CreateChannel(wordSearchServerAddr,
                                 grpc::InsecureChannelCredentials());

  _feature.reset(new Feature());

  return true;
}

grpc::Status FeatureServer::onQuery(grpc::ServerContext *context,
                                    const proto::QueryMessage *request,
                                    proto::Empty *response) {
  std::vector<uchar> data(request->image().begin(), request->image().end());

  assert(data.size() > 0);
  const bool copyData = false;
  cv::Mat image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);
  assert(image.type() == CV_8U);
  assert(image.channels() == 1);

  double fx = request->cameramodel().fx();
  double fy = request->cameramodel().fy();
  double cx = request->cameramodel().cx();
  double cy = request->cameramodel().cy();
  int width = image.cols;
  int height = image.rows;
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
  session.perspectiveStart = request->session().perspectivestart();
  session.perspectiveEnd = request->session().perspectiveend();

  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  session.featuresStart = Utility::getTime();
  _feature->extract(image, keyPoints, descriptors);
  session.featuresEnd = Utility::getTime();

  WordSearchClient client(_channel);
  client.onFeature(keyPoints, descriptors, camera, session);

  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void FeatureServer::run(std::string featureServerAddr) {
  std::string server_address(featureServerAddr);

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
