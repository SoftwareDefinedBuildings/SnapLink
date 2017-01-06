#include "HTTPFrontEndServer.h"
#include "front_end/FeatureClient.h"
#include "util/Utility.h"
#include <cstdlib>
#include <cstring>

bool HTTPFrontEndServer::init(std::string featureServerAddr, uint16_t port,
                              unsigned int maxClients) {
  _httpFront.reset(new HTTPFrontEnd());
  _gen = std::mt19937(std::random_device()());
  _channel = grpc::CreateChannel(featureServerAddr,
                                 grpc::InsecureChannelCredentials());

  bool success = _httpFront->start(port, maxClients);
  if (success) {
    _httpFront->registerOnQuery(std::bind(&HTTPFrontEndServer::onQuery, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  }

  return true;
}

grpc::Status
HTTPFrontEndServer::onDetection(grpc::ServerContext *context,
                                const proto::DetectionMessage *request,
                                proto::Empty *response) {
  assert(request->session().type() == proto::Session::HTTP_POST);

  std::unique_ptr<Session> session(new Session());
  session->id = request->session().id();
  session->type = HTTP_POST;
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

void HTTPFrontEndServer::run(std::string httpServerAddr) {
  std::string server_address(httpServerAddr);

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
HTTPFrontEndServer::onQuery(std::unique_ptr<cv::Mat> &&image,
                            std::unique_ptr<CameraModel> &&camera) {
  std::unique_ptr<Session> session(new Session);
  session->overallStart = Utility::getTime(); // log start of processing
  session->type = HTTP_POST;

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
