#include "front/HTTPServer.h"
#include "data/CameraModel.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "event/QueryEvent.h"
#include "process/Identification.h"
#include "util/Time.h"
#include <QCoreApplication>
#include <cstdlib>
#include <string.h>
#include <strings.h>

const std::string HTTPServer::none = "None";

HTTPServer::HTTPServer()
    : _daemon(nullptr), _numClients(0), _gen(std::random_device()()) {}

HTTPServer::~HTTPServer() {
  stop();
  _numClients = 0;
}

bool HTTPServer::start(uint16_t port, unsigned int maxClients) {
  _maxClients = maxClients;

  // start MHD daemon, listening on port
  unsigned int flags = MHD_USE_SELECT_INTERNALLY | MHD_USE_EPOLL_LINUX_ONLY;
  struct MHD_OptionItem ops[] = {
      {MHD_OPTION_THREAD_POOL_SIZE, _maxClients, nullptr},
      {MHD_OPTION_NOTIFY_COMPLETED, &requestCompleted,
       static_cast<void *>(this)},
      {MHD_OPTION_END, 0, NULL}};
  _daemon = MHD_start_daemon(flags, port, nullptr, nullptr,                //
                             &answerConnection, static_cast<void *>(this), //
                             MHD_OPTION_ARRAY, ops,                        //
                             MHD_OPTION_END);
  if (_daemon == nullptr) {
    return false;
  }

  return true;
}

void HTTPServer::stop() {
  if (_daemon != nullptr) {
    MHD_stop_daemon(_daemon);
    _daemon = nullptr;
  }
}

void HTTPServer::registerOnQuery(OnQueryFunction onQuery) {
  _onQuery = onQuery;
}

// bool HTTPServer::event(QEvent *event) {
//   if (event->type() == DetectionEvent::type()) {
//     DetectionEvent *detectionEvent = static_cast<DetectionEvent *>(event);
//     std::unique_ptr<Session> session = detectionEvent->takeSession();
//     // find() const is thread-safe
//     const auto iter = _connInfoMap.find(session->id);
//     ConnectionInfo *connInfo = iter->second;
//     connInfo->names = detectionEvent->takeNames();
//     connInfo->session = std::move(session);
//     connInfo->detected.release();
//     return true;
//   } else if (event->type() == FailureEvent::type()) {
//     FailureEvent *failureEvent = static_cast<FailureEvent *>(event);
//     std::unique_ptr<Session> session = failureEvent->takeSession();
//     // find() const is thread-safe
//     const auto iter = _connInfoMap.find(session->id);
//     ConnectionInfo *connInfo = iter->second;
//     connInfo->session = std::move(session);
//     connInfo->detected.release();
//     return true;
//   }
//   return QObject::event(event);
// }
//
int HTTPServer::answerConnection(void *cls, struct MHD_Connection *connection,
                                 const char *url, const char *method,
                                 const char *version, const char *uploadData,
                                 size_t *uploadDataSize, void **conCls) {
  if (strcasecmp(method, MHD_HTTP_METHOD_POST) != 0) {
    return respond(connection, none, MHD_HTTP_BAD_REQUEST);
  }

  HTTPServer *httpServer = static_cast<HTTPServer *>(cls);
  assert(httpServer != nullptr);

  if (*conCls == nullptr) // new connection
  {
    httpServer->_mutex.lock();
    if (httpServer->_numClients >= httpServer->_maxClients) {
      httpServer->_mutex.unlock();
      return respond(connection, none, MHD_HTTP_SERVICE_UNAVAILABLE);
    }
    httpServer->_numClients++;
    httpServer->_mutex.unlock();

    ConnectionInfo *connInfo = new ConnectionInfo();
    assert(connInfo != nullptr);

    connInfo->session.reset(new Session());
    connInfo->session->type = HTTP_POST;

    // reserve enough space for an image
    connInfo->rawData.reset(new std::vector<char>());
    connInfo->rawData->reserve(IMAGE_INIT_SIZE);

    connInfo->postProcessor =
        MHD_create_post_processor(connection, POST_BUFFER_SIZE, iteratePost,
                                  static_cast<void *>(connInfo));
    if (connInfo->postProcessor == nullptr) {
      httpServer->_numClients--; // _numClients is atomic
      return MHD_NO;
    }

    httpServer->_mutex.lock();
    connInfo->session->id = httpServer->_dis(httpServer->_gen);
    httpServer->_connInfoMap.insert(
        std::make_pair(connInfo->session->id, connInfo));
    httpServer->_mutex.unlock();

    connInfo->sessionType = POST;

    *conCls = static_cast<void *>(connInfo);

    return MHD_YES;
  }

  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*conCls);

  if (*uploadDataSize != 0) {
    MHD_post_process(connInfo->postProcessor, uploadData, *uploadDataSize);
    *uploadDataSize = 0;

    return MHD_YES;
  } else {
    // all data are received
    if (!connInfo->rawData->empty()) {
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
        return respond(connection, none, MHD_HTTP_BAD_REQUEST);
      }

      // blocking wait
      connInfo->names = _onQuery(std::move(image), std::move(camera),
                                 std::move(connInfo->session));
    }

    std::string answer = none;
    if (connInfo->names != nullptr && !connInfo->names->empty()) {
      answer = std::move(connInfo->names->at(0));
    }

    return respond(connection, answer, MHD_HTTP_OK);
  }

  return respond(connection, none, MHD_HTTP_BAD_REQUEST);
}

int HTTPServer::iteratePost(void *coninfo_cls, enum MHD_ValueKind kind,
                            const char *key, const char *filename,
                            const char *contentType,
                            const char *transferEncoding, const char *data,
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
                                  void **conCls,
                                  enum MHD_RequestTerminationCode toe) {
  HTTPServer *httpServer = static_cast<HTTPServer *>(cls);
  assert(httpServer != nullptr);
  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*conCls);
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
    std::cout << "TAG_TIME perspective "
              << session->perspectiveEnd - session->perspectiveStart << " ms"
              << std::endl;
  }

  if (connInfo->postProcessor != nullptr) {
    MHD_destroy_post_processor(connInfo->postProcessor);
    httpServer->_numClients--; // _numClients is atomic
  }

  httpServer->_mutex.lock();
  httpServer->_connInfoMap.erase(session->id);
  httpServer->_mutex.unlock();

  delete connInfo;
  connInfo = nullptr;
  *conCls = nullptr;
}

int HTTPServer::respond(struct MHD_Connection *connection,
                        const std::string &message, int status_code) {
  struct MHD_Response *response = MHD_create_response_from_buffer(
      message.length(),
      const_cast<void *>(static_cast<const void *>(message.c_str())),
      MHD_RESPMEM_MUST_COPY);
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

  // imwrite("image.jpg", image);

  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}
