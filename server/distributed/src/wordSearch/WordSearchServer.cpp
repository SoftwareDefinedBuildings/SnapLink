#include "WordSearchServer.h"
#include "PerspectiveClient.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/WordsKdTree.h"
#include "util/Time.h"
#include <QDebug>

bool WordSearchServer::init(std::string perspectiveServerAddr,
                            std::vector<std::string> dbfiles) {
  _channel = grpc::CreateChannel(perspectiveServerAddr,
                                 grpc::InsecureChannelCredentials());

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _wordSearch.reset(new WordSearch(std::move(words)));

  return true;
}

grpc::Status WordSearchServer::onFeature(grpc::ServerContext *context,
                                         const proto::FeatureMessage *request,
                                         proto::Empty *response) {
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

  assert(request->descriptors_size() > 0);
  int descriptorSize = request->descriptors(0).values_size();
  assert(descriptorSize > 0);
  cv::Mat descriptors(request->descriptors_size(),
                      request->descriptors(0).values_size(), CV_32F);
  for (int row = 0; row < request->descriptors_size(); row++) {
    assert(request->descriptors(row).values_size() == descriptorSize);
    for (int col = 0; col < descriptorSize; col++) {
      descriptors.at<float>(row, col) = request->descriptors(row).values(col);
    }
  }
  assert(descriptors.type() == CV_32F);
  assert(descriptors.channels() == 1);

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
  session.perspectiveStart = request->session().perspectivestart();
  session.perspectiveEnd = request->session().perspectiveend();

  session.wordsStart = getTime();
  std::vector<int> wordIds = _wordSearch->search(descriptors);
  session.wordsEnd = getTime();
  PerspectiveClient client(_channel);
  client.onWord(wordIds, keyPoints, camera, session);  
  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void WordSearchServer::run(std::string wordSearchServerAddr) {
  std::string server_address(wordSearchServerAddr);

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
