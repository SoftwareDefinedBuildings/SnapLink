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
grpc::Status GrpcFrontEnd::getModels(
    grpc::ServerContext *context,
    const cellmate_grpc::Empty *empty,
    cellmate_grpc::Models *models) {
  (void)context; // ignore that variable without causing warnings
  
  std::map<int, std::vector<Label>> labels = this->getOnGetLabels()();
  for (std::map<int, std::vector<Label>>::iterator it = labels.begin(); 
          it!=labels.end(); ++it) {
    int roomId = it->first;
    std::vector<Label> labelsInRoom = it->second;
    cellmate_grpc::Model *model = models->add_models();;
    model->set_id(roomId);
    for(auto singleLabel : labelsInRoom) {
      cellmate_grpc::Label* label = model->add_labels();
      label->set_name(singleLabel.getName());
      label->set_x(singleLabel.getPoint3().x); 
      label->set_y(singleLabel.getPoint3().y);
      label->set_z(singleLabel.getPoint3().z);
      label->set_roomid(singleLabel.getRoomId());
    }
  }
  return grpc::Status::OK;
}
grpc::Status GrpcFrontEnd::onClientQuery(
    grpc::ServerContext *context,
    grpc::ServerReaderWriter<cellmate_grpc::ServerRespondMessage,
                             cellmate_grpc::ClientQueryMessage> *stream) {
  (void)context; // ignore that variable without causing warnings
  cellmate_grpc::ClientQueryMessage request;
  while (stream->Read(&request)) {
    cellmate_grpc::ServerRespondMessage response;
    response.set_id(request.id());
    {
      std::lock_guard<std::mutex> lock(_mutex);
      if (this->_numClients >= this->_maxClients) {
        response.add_name("Too many clients, server is busy");
        response.add_x(-1);
        response.add_y(-1);
        response.set_width(-1);
        response.set_height(-1);
        stream->Write(response);
        continue;
      }
      this->_numClients++;
    }
    std::vector<uchar> data(request.image().begin(), request.image().end());
    assert(data.size() > 0);
    bool copyData = false;

    cv::Mat imageUnrotated =
        imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);
    //     imwrite("imageUnrotated.jpg", imageUnrotated);
    cv::Mat image = rotateClockwise(imageUnrotated, request.angle());
    //     imwrite("imageRotated.jpg", image);

    if (image.empty() || image.type() != CV_8U || image.channels() != 1) {
      response.add_name("Invalid query data");
      response.add_x(-1);
      response.add_y(-1);
      response.set_width(-1);
      response.set_height(-1);
      stream->Write(response);
      continue;
    }

    int width = image.cols;
    int height = image.rows;

    double fx, fy, cx, cy;
    setIntrinsics(width, height, request.angle(), fx, fy, cx, cy, request);
    std::cout << "Width = " << width << ", Height = " << height
              << " Cx = " << cx << " Cy = " << cy << std::endl;

    CameraModel camera("", fx, fy, cx, cy, cv::Size(width, height));
    std::pair<Transform, std::vector<FoundItem>> result;

    result = this->getOnIdentify()(image, camera);
    Transform imagePose = result.first;
    std::vector<FoundItem> results = result.second;
    rotateBack(results, request.angle(), width, height);
    this->_numClients--;
    if (!imagePose.isNull()) {
      response.set_poseavailable(true);
      response.set_r11(imagePose.r11());
      response.set_r12(imagePose.r12());
      response.set_r13(imagePose.r13());
      response.set_r21(imagePose.r21());
      response.set_r22(imagePose.r22());
      response.set_r23(imagePose.r23());
      response.set_r31(imagePose.r31());
      response.set_r32(imagePose.r32());
      response.set_r33(imagePose.r33());
      response.set_tx(imagePose.x());
      response.set_ty(imagePose.y());
      response.set_tz(imagePose.z());
    } else {
      response.set_poseavailable(false);
    }
    if (!results.empty()) {
      for(unsigned int i = 0; i < results.size(); i++) {
        response.add_name(results[i].name());
        response.add_x(results[i].x());
        response.add_y(results[i].y());
        response.add_size(results[i].size());
      }
      response.set_width(width > height ? height : width);
      response.set_height(width > height ? width : height);
      response.set_roomid(results[0].roomId());
    } else {
      response.add_name(none);
      response.add_x(-1);
      response.add_y(-1);
      response.set_width(width > height ? height : width);
      response.set_height(width > height ? width : height);
    }
    stream->Write(response);
  }
  return grpc::Status::OK;
}

cv::Mat GrpcFrontEnd::rotateClockwise(cv::Mat src, double angle) {
  cv::Mat dst;
  if (angle == 90) {
    cv::transpose(src, dst);
    cv::flip(dst, dst, 1);
  } else if (angle == 180) {
    cv::flip(src, dst, -1);
  } else if (angle == 270) {
    cv::transpose(src, dst);
    cv::flip(dst, dst, 0);
  } else {
    // angle = 0, no need to rotation
    dst = src;
  }
  return dst;
}

void GrpcFrontEnd::rotateBack(std::vector<FoundItem> &results, double angle,
                              int width, int height) {
  for (unsigned int i = 0; i < results.size(); i++) {
    double oldX = results[i].x();
    double oldY = results[i].y();
    if (oldX == -1) {
      continue;
    }
    if (angle == 90) {
      // do nothing
    } else if (angle == 180) {
      results[i].setX(oldY);
      results[i].setY(width - oldX);
    } else if (angle == 270) {
      results[i].setX(width - oldX);
      results[i].setY(height - oldY);

    } else {
      // angle = 0
      results[i].setX(height - oldY);
      results[i].setY(oldX);
    }
    std::cout << "Angle is " << angle << std::endl;
    std::cout << "After rotateback, x = " << results[i].x()
              << " y = " << results[i].y() << std::endl;
  }
}

void GrpcFrontEnd::setIntrinsics(double width, double height, double angle,
                                 double &fx, double &fy, double &cx, double &cy,
                                 cellmate_grpc::ClientQueryMessage &request) {
  fx = request.fx();
  fy = request.fy();
  if (angle == 0) {
    cx = request.cx();
    cy = request.cy();
  } else if (angle == 90) {
    cx = width - request.cy();
    cy = request.cx();
  } else if (angle == 180) {
    cx = width - request.cx();
    cy = height - request.cy();
  } else if (angle == 270) {
    cx = request.cy();
    cy = height - request.cx();
  }
}
