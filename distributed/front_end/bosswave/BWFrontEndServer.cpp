#include "BWFrontEndServer.h"
#include "front_end/FeatureClient.h"
#include "util/Utility.h"
#include <cstdlib>
#include <cstring>

bool BWFrontEndServer::init(std::string featureServerAddr,
                            unsigned int maxClients) {
  _bwFront.reset(new BWFrontEnd());
  _gen = std::mt19937(std::random_device()());
  _channel = grpc::CreateChannel(featureServerAddr,
                                 grpc::InsecureChannelCredentials());

  bool success = _bwFront->start(maxClients);
  if (success) {
    _bwFront->registerOnQuery(std::bind(&BWFrontEndServer::onQuery, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
  }

  return true;
}

grpc::Status
BWFrontEndServer::onDetection(grpc::ServerContext *context,
                              const proto::DetectionMessage *request,
                              proto::Empty *response) {
  assert(request->session().type() == proto::Session::BOSSWAVE);

  std::unique_ptr<Session> session(new Session());
  session->id = request->session().id();
  session->type = BOSSWAVE;
  session->overallStart = request->session().overallstart();
  session->overallEnd = request->session().overallend();
  session->featuresStart = request->session().featuresstart();
  session->featuresEnd = request->session().featuresend();
  session->wordsStart = request->session().wordsstart();
  session->wordsEnd = request->session().wordsend();
  session->perspectiveStart = request->session().perspectivestart();
  session->perspectiveEnd = request->session().perspectiveend();

  std::unique_ptr<std::vector<std::string>> names(
      new std::vector<std::string>());
  for (auto name : request->names()) {
    names->emplace_back(name);
  }

  _mutex.lock();
  auto iter = _sessionMap.find(session->id);
  std::unique_ptr<SessionData> &sessionData = iter->second;
  sessionData->session = std::move(session);
  sessionData->names = std::move(names);
  QSemaphore &detected = sessionData->detected;
  _mutex.unlock();

  detected.release();

  return grpc::Status::OK;
}

void BWFrontEndServer::run(QString bwServerAddr) {
  std::string server_address(bwServerAddr.toStdString());

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

std::vector<std::string>
BWFrontEndServer::onQuery(std::unique_ptr<cv::Mat> &&image,
                          std::unique_ptr<CameraModel> &&camera) {

  std::cout << "onQuery" << std::endl;

  std::unique_ptr<Session> session(new Session);
  session->overallStart = Utility::getTime(); // log start of processing
  session->type = BOSSWAVE;

  std::unique_ptr<SessionData> sessionData(new SessionData);
  QSemaphore &detected = sessionData->detected;

  _mutex.lock();
  long id = _dis(_gen); // this is not thread safe
  _sessionMap.emplace(id, std::move(sessionData));
  _mutex.unlock();

  session->id = id;

  FeatureClient client(_channel);
  client.onQuery(*image, *camera, *session);

  // TODO use condition variable?
  detected.acquire();

  _mutex.lock();
  auto iter = _sessionMap.find(id);
  sessionData = std::move(iter->second);
  _sessionMap.erase(id);
  _mutex.unlock();

  assert(sessionData != nullptr);
  assert(sessionData->session != nullptr);

  session = std::move(sessionData->session);

  // print time
  session->overallEnd = Utility::getTime(); // log processing end time
  std::cout << "Time overall: " << session->overallEnd - session->overallStart
            << " ms" << std::endl;
  std::cout << "Time features: "
            << session->featuresEnd - session->featuresStart << " ms"
            << std::endl;
  std::cout << "Time words: " << session->wordsEnd - session->wordsStart
            << " ms" << std::endl;
  std::cout << "Time perspective: "
            << session->perspectiveEnd - session->perspectiveStart << " ms"
            << std::endl;

  return *(sessionData->names);
}
