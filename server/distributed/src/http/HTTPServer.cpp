#include "http/HTTPServer.h"
#include "data/CameraModel.h"
#include "http/CellMateClient.h"
#include "message/DetectionEvent.h"
#include "message/FailureEvent.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <cstdlib>
#include <string.h>
#include <strings.h>

// Take in the "service" instance (in this case representing an asynchronous
// server) and the completion queue "cq" used for asynchronous communication
// with the gRPC runtime.
HTTPServer::CallData::CallData(proto::Http::AsyncService *service,
                               grpc::ServerCompletionQueue *cq,
                               std::map<long, ConnectionInfo *> &connInfoMap)
    : service_(service), cq_(cq), responder_(&ctx_), status_(CREATE),
      _connInfoMap(connInfoMap) {
  // Invoke the serving logic right away.
  proceed();
}

void HTTPServer::CallData::proceed() {
  if (status_ == CREATE) {
    // Make this instance progress to the PROCESS state.
    status_ = PROCESS;

    // As part of the initial CREATE state, we *request* that the system
    // start processing SayHello requests. In this request, "this" acts are
    // the tag uniquely identifying the request (so that different CallData
    // instances can serve different requests concurrently), in this case
    // the memory address of this CallData instance.
    service_->RequestOnDetection(&ctx_, &_detection, &responder_, cq_, cq_,
                                 this);
  } else if (status_ == PROCESS) {
    // Spawn a new CallData instance to serve new clients while we process
    // the one for this CallData. The instance will deallocate itself as
    // part of its FINISH state.
    new CallData(service_, cq_, _connInfoMap);

    // The actual processing.
    std::vector<std::string> names;
    for (int i = 0; i < _detection.names_size(); i++) {
      names.emplace_back(_detection.names(i));
    }

    Session session;
    session.id = _detection.session().id();
    if (_detection.session().type() == proto::Session::HTTP_POST) {
      session.type = HTTP_POST;
    } else if (_detection.session().type() == proto::Session::BOSSWAVE) {
      session.type = BOSSWAVE;
    }
    session.overallStart = _detection.session().overallstart();
    session.overallEnd = _detection.session().overallend();
    session.featuresStart = _detection.session().featuresstart();
    session.featuresEnd = _detection.session().featuresend();
    session.wordsStart = _detection.session().wordsstart();
    session.wordsEnd = _detection.session().wordsend();
    session.signaturesStart = _detection.session().signaturesstart();
    session.signaturesEnd = _detection.session().signaturesend();
    session.perspectiveStart = _detection.session().perspectivestart();
    session.perspectiveEnd = _detection.session().perspectiveend();

    // find() const is thread-safe
    const auto iter = _connInfoMap.find(session.id);
    ConnectionInfo *connInfo = iter->second;
    connInfo->names.reset(new std::vector<std::string>(std::move(names)));
    connInfo->session.reset(new Session(std::move(session)));
    connInfo->detected.release();

    // And we are done! Let the gRPC runtime know we've finished, using the
    // memory address of this instance as the uniquely identifying tag for
    // the event.
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    // Once in the FINISH state, deallocate ourselves (CallData).
    delete this;
  }
}

const std::string HTTPServer::busypage =
    "This server is busy, please try again later.";
const std::string HTTPServer::errorpage = "This doesn't seem to be right.";

HTTPServer::HTTPServer()
    : _daemon(nullptr), _numClients(0), _gen(std::random_device()()),
      _channel(grpc::CreateChannel("localhost:50051",
                                   grpc::InsecureChannelCredentials())) {}

HTTPServer::~HTTPServer() {
  if (_daemon != nullptr) {
    MHD_stop_daemon(_daemon);
    _daemon = nullptr;
  }
  _numClients = 0;
  server_->Shutdown();
  // Always shutdown the completion queue after the server.
  cq_->Shutdown();
}

bool HTTPServer::startMHD(uint16_t port, unsigned int maxClients) {
  _maxClients = maxClients;

  // start MHD daemon, listening on port
  unsigned int flags = MHD_USE_SELECT_INTERNALLY | MHD_USE_EPOLL_LINUX_ONLY;
  _daemon = MHD_start_daemon(flags, port, nullptr, nullptr, &answerConnection,
                             static_cast<void *>(this),
                             MHD_OPTION_NOTIFY_COMPLETED, &requestCompleted,
                             static_cast<void *>(this), MHD_OPTION_END);
  if (_daemon == nullptr) {
    return false;
  }

  return true;
}

void HTTPServer::runGRPC() {
  std::string server_address("0.0.0.0:50051");

  grpc::ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service_" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *asynchronous* service.
  builder.RegisterService(&service_);
  // Get hold of the completion queue used for the asynchronous communication
  // with the gRPC runtime.
  cq_ = builder.AddCompletionQueue();
  // Finally assemble the server.
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  // Proceed to the server's main loop.
  handleRpcs();
}

int HTTPServer::getMaxClients() const { return _maxClients; }

int HTTPServer::getNumClients() const { return _numClients; }

void HTTPServer::setNumClients(int numClients) { _numClients = numClients; }

int HTTPServer::answerConnection(void *cls, struct MHD_Connection *connection,
                                 const char *url, const char *method,
                                 const char *version, const char *upload_data,
                                 size_t *upload_data_size, void **con_cls) {
  if (strcasecmp(method, MHD_HTTP_METHOD_POST) != 0) {
    return sendPage(connection, errorpage, MHD_HTTP_BAD_REQUEST);
  }

  HTTPServer *httpServer = static_cast<HTTPServer *>(cls);
  assert(httpServer != nullptr);

  if (*con_cls == nullptr) // new connection
  {
    if (httpServer->getNumClients() >= httpServer->getMaxClients()) {
      return sendPage(connection, busypage, MHD_HTTP_SERVICE_UNAVAILABLE);
    }

    ConnectionInfo *connInfo = new ConnectionInfo();
    assert(connInfo != nullptr);

    connInfo->session.reset(new Session());
    connInfo->session->id = httpServer->_dis(httpServer->_gen);
    connInfo->session->type = HTTP_POST;

    httpServer->_mutex.lock();
    httpServer->_connInfoMap.insert(
        std::make_pair(connInfo->session->id, connInfo));
    httpServer->_mutex.unlock();

    // reserve enough space for an image
    connInfo->rawData.reset(new std::vector<char>());
    connInfo->rawData->reserve(IMAGE_INIT_SIZE);

    connInfo->postProcessor =
        MHD_create_post_processor(connection, POST_BUFFER_SIZE, iteratePost,
                                  static_cast<void *>(connInfo));
    if (connInfo->postProcessor == nullptr) {
      return MHD_NO;
    }

    httpServer->setNumClients(httpServer->getNumClients() + 1);

    connInfo->sessionType = POST;

    *con_cls = static_cast<void *>(connInfo);

    return MHD_YES;
  }

  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*con_cls);

  if (*upload_data_size != 0) {
    MHD_post_process(connInfo->postProcessor, upload_data, *upload_data_size);
    *upload_data_size = 0;

    return MHD_YES;
  } else {
    if (!connInfo->rawData->empty()) {
      // all data are received
      connInfo->session->overallStart = getTime(); // log start of processing

      double fx = connInfo->cameraInfo.fx;
      double fy = connInfo->cameraInfo.fy;
      double cx = connInfo->cameraInfo.cx;
      double cy = connInfo->cameraInfo.cy;
      std::unique_ptr<cv::Mat> image(new cv::Mat());
      std::unique_ptr<CameraModel> camera(new CameraModel());
      createData(*(connInfo->rawData), fx, fy, cx, cy, *image, *camera);
      if (image->empty()) {
        // TODO do I need to free anything here?
        return sendPage(connection, errorpage, MHD_HTTP_BAD_REQUEST);
      }

      CellMateClient client(httpServer->_channel);
      client.query(*(connInfo->rawData), *camera, *(connInfo->session));
      // client.finish(); // we do not care about the return
    }

    // wait for the result to come
    connInfo->detected.acquire();

    std::string answer = "None";
    if (connInfo->names != nullptr && !connInfo->names->empty()) {
      answer = std::move(connInfo->names->at(0));
    }

    return sendPage(connection, answer, MHD_HTTP_OK);
  }

  return sendPage(connection, errorpage, MHD_HTTP_BAD_REQUEST);
}

int HTTPServer::iteratePost(void *coninfo_cls, enum MHD_ValueKind kind,
                            const char *key, const char *filename,
                            const char *content_type,
                            const char *transfer_encoding, const char *data,
                            uint64_t off, size_t size) {
  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(coninfo_cls);
  assert(connInfo != nullptr);

  if (strcmp(key, "file") != 0 && strcmp(key, "fx") != 0 &&
      strcmp(key, "fy") != 0 && strcmp(key, "cx") != 0 &&
      strcmp(key, "cy") != 0) {
    return MHD_NO;
  }

  if (size > 0) {
    if (strcmp(key, "file") == 0) {
      connInfo->rawData->insert(connInfo->rawData->end(), data, data + size);
    } else if (strcmp(key, "fx") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cameraInfo.fx = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "fy") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cameraInfo.fy = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "cx") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cameraInfo.cx = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "cy") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cameraInfo.cy = atof(buf);
      delete[] buf;
    }
  }

  return MHD_YES;
}

void HTTPServer::requestCompleted(void *cls, struct MHD_Connection *connection,
                                  void **con_cls,
                                  enum MHD_RequestTerminationCode toe) {
  HTTPServer *httpServer = static_cast<HTTPServer *>(cls);
  assert(httpServer != nullptr);
  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*con_cls);
  assert(connInfo != nullptr);

  std::unique_ptr<Session> session = std::move(connInfo->session);
  if (session != nullptr) {
    session->overallEnd = getTime(); // log processing end time

    std::cout << "TAG_TIME overall "
              << session->overallEnd - session->overallStart << " ms"
              << std::endl;
    std::cout << "TAG_TIME features "
              << session->featuresEnd - session->featuresStart << " ms"
              << std::endl;
    std::cout << "TAG_TIME words " << session->wordsEnd - session->wordsStart
              << " ms" << std::endl;
    std::cout << "TAG_TIME signatures "
              << session->signaturesEnd - session->signaturesStart << " ms"
              << std::endl;
    std::cout << "TAG_TIME perspective "
              << session->perspectiveEnd - session->perspectiveStart << " ms"
              << std::endl;
  }

  if (connInfo->postProcessor != nullptr) {
    MHD_destroy_post_processor(connInfo->postProcessor);
    httpServer->setNumClients(httpServer->getNumClients() - 1);
  }

  httpServer->_mutex.lock();
  httpServer->_connInfoMap.erase(session->id);
  httpServer->_mutex.unlock();

  delete connInfo;
  connInfo = nullptr;
  *con_cls = nullptr;
}

int HTTPServer::sendPage(struct MHD_Connection *connection,
                         const std::string &page, int status_code) {
  struct MHD_Response *response = MHD_create_response_from_buffer(
      page.length(),
      const_cast<void *>(static_cast<const void *>(page.c_str())),
      MHD_RESPMEM_PERSISTENT);
  if (!response) {
    return MHD_NO;
  }

  int ret = MHD_queue_response(connection, status_code, response);
  MHD_destroy_response(response);

  return ret;
}

void HTTPServer::createData(const std::vector<char> &data, double fx, double fy,
                            double cx, double cy, cv::Mat &image,
                            CameraModel &camera) {
  // no data copy is needed because conn info
  const bool copyData = false;
  image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}

// This can be run in multiple threads if needed.
void HTTPServer::handleRpcs() {
  // Spawn a new CallData instance to serve new clients.
  new CallData(&service_, cq_.get(), _connInfoMap);
  void *tag; // uniquely identifies a request.
  bool ok;
  while (true) {
    // Block waiting to read the next event from the completion queue. The
    // event is uniquely identified by its tag, which in this case is the
    // memory address of a CallData instance.
    // The return value of Next should always be checked. This return value
    // tells us whether there is any kind of event or cq_ is shutting down.
    GPR_ASSERT(cq_->Next(&tag, &ok));
    GPR_ASSERT(ok);
    static_cast<CallData *>(tag)->proceed();
  }
}
