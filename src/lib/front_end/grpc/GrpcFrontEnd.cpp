#include "lib/front_end/grpc/GrpcFrontEnd.h"
#include "lib/data/CameraModel.h"
#include <opencv2/core/core.hpp>
#include <string>
const std::string GrpcFrontEnd::none = "None";
GrpcFrontEnd::GrpcFrontEnd(int grpcServerAddr, unsigned int maxClients) {
  _numClients = 0;
  _serverAddress = std::to_string(grpcServerAddr);
  _maxClients = maxClients; 
}


GrpcFrontEnd::~GrpcFrontEnd() {
  std::cerr << "GrpcFrontEnd destructor" << std::endl;
  stop();
}

bool GrpcFrontEnd::start() {
  std::cerr << "DEBUG: Grpc start()" << std::endl;
  this->moveToThread(&_thread);
  connect(&_thread, &QThread::started, this, &GrpcFrontEnd::run);
  _thread.start();
  return true;
}

void GrpcFrontEnd::run() {
  std::string server_address(_serverAddress);

  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort("0.0.0.0:"+server_address, grpc::InsecureServerCredentials());
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
void GrpcFrontEnd::stop() {
  _numClients = 0;
  _serverAddress = "";
  _maxClients = 0;
}

grpc::Status GrpcFrontEnd::onClientQuery(grpc::ServerContext *context,
                                         const cellmate_grpc::ClientQueryMessage *request,
                                         cellmate_grpc::ServerRespondMessage *response) {

  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (this->_numClients >= this->_maxClients) {
      response->set_foundname("Too many clients, server is busy");
      return grpc::Status::OK;
    }
    this->_numClients++;
  }
  std::vector<uchar> data(request->image().begin(), request->image().end());
  assert(data.size() > 0);
  bool copyData = false;
  cv::Mat image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);
  imwrite("image.jpg", image);
  assert(image.type() == CV_8U);
  assert(image.channels() == 1);
  double fx = request->fx();
  double fy = request->fy();
  double cx = request->cx();
  double cy = request->cy();
  int width = image.cols;
  int height = image.rows;
  CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));
  std::vector<FoundItem> results;
  results = this->getOnQuery()(image, camera);
  
  
  this->_numClients--;  
  std::string result = none;
  if (!results.empty()) {
    result = std::move(results.at(0).name());
  }
  response->set_foundname(result);
  return grpc::Status::OK;
}



