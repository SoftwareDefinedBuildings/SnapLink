#include "lib/front_end/grpc/GrpcFrontEnd.h"
#include "lib/data/CameraModel.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
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
  builder.AddListeningPort("0.0.0.0:" + server_address,
                           grpc::InsecureServerCredentials());
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

grpc::Status GrpcFrontEnd::onClientQuery(
    grpc::ServerContext *context,
    grpc::ServerReaderWriter<cellmate_grpc::ServerRespondMessage,
                             cellmate_grpc::ClientQueryMessage> *stream) {
  (void)context; // ignore that variable without causing warnings
  cellmate_grpc::ClientQueryMessage request;
  while (stream->Read(&request)) {
    cellmate_grpc::ServerRespondMessage response;
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (this->_numClients >= this->_maxClients) {
        response.set_name("Too many clients, server is busy");
        response.set_x(-1);
        response.set_y(-1);
        stream->Write(response);
        continue;
      }
      this->_numClients++;
    }
    std::vector<uchar> data(request.image().begin(), request.image().end());
    assert(data.size() > 0);
    bool copyData = false;
    cv::Mat image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

    if (image.empty() || image.type() != CV_8U || image.channels() != 1) {
      response.set_name("Invalid query data");
      response.set_x(-1);
      response.set_y(-1);
      stream->Write(response);
      continue;
    }

    int width = image.cols;
    int height = image.rows;

    double fx = request.fx();
    double fy = request.fy();
    double cx = request.cx();
    double cy = request.cy();
    std::unique_ptr<CameraModel> camera;
    if (fx > 0 && fy > 0 && cx >= 0 && cy >= 0) {
      camera.reset(
          new CameraModel("", fx, fy, cx, cy, cv::Size(width, height)));
    }
    std::vector<FoundItem> results;
    results = this->getOnQuery()(image, camera.get());

    this->_numClients--;
    if (!results.empty()) {
      response.set_name(results[0].name());
      response.set_x(results[0].x());
      response.set_y(results[0].y());
      response.set_width(results[0].width());
    } else {
      response.set_name(none);
      response.set_x(-1);
      response.set_y(-1);
    }
    stream->Write(response);
  }
  return grpc::Status::OK;
}
