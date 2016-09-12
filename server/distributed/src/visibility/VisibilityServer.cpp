#include "VisibilityServer.h"
#include "HTTPClient.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/CameraModel.h"
#include "data/LabelsSimple.h"
#include "data/Session.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "util/Time.h"
#include <QDebug>

bool VisibilityServer::init(std::string frontServerAddr,
                            std::vector<std::string> dbfiles) {
  _channel =
      grpc::CreateChannel(frontServerAddr, grpc::InsecureChannelCredentials());

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return false;
  }

  _visibility.reset(new Visibility(std::move(labels)));

  return true;
}

grpc::Status VisibilityServer::onLocation(grpc::ServerContext *context,
                                          const proto::LocationMessage *request,
                                          proto::Empty *response) {
  int dbId = request->dbid();

  double fx = request->cameramodel().fx();
  double fy = request->cameramodel().fy();
  double cx = request->cameramodel().cx();
  double cy = request->cameramodel().cy();
  int width = request->cameramodel().width();
  int height = request->cameramodel().height();
  CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));

  float r11 = request->pose().r11();
  float r12 = request->pose().r12();
  float r13 = request->pose().r13();
  float r21 = request->pose().r21();
  float r22 = request->pose().r22();
  float r23 = request->pose().r23();
  float r31 = request->pose().r31();
  float r32 = request->pose().r32();
  float r33 = request->pose().r33();
  float x = request->pose().x();
  float y = request->pose().y();
  float z = request->pose().z();
  Transform pose(r11, r12, r13, x, //
                 r21, r22, r23, y, //
                 r31, r32, r33, z);

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

  std::vector<std::string> names = _visibility->process(dbId, camera, pose);

  HTTPClient client(_channel);
  client.onDetection(names, session);

  return grpc::Status::OK;
}

// There is no shutdown handling in this code.
void VisibilityServer::run(std::string visibilityServerAddr) {
  std::string server_address(visibilityServerAddr);

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
