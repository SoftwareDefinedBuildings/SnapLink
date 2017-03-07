#include "lib/front_end/http/HTTPFrontEnd.h"
#include <cstdlib>
#include <string.h>
#include <strings.h>

const std::string HTTPFrontEnd::none = "None";

HTTPFrontEnd::HTTPFrontEnd(uint16_t port, unsigned int maxClients)
    : _port(port), _daemon(nullptr), _numClients(0), _maxClients(maxClients) {}

HTTPFrontEnd::~HTTPFrontEnd() {
  std::cerr << "HTTPFrontEnd destructor" << std::endl;
  stop();
}

bool HTTPFrontEnd::start() {
  // start MHD daemon, listening on port
  unsigned int flags = MHD_USE_SELECT_INTERNALLY | MHD_USE_EPOLL_LINUX_ONLY;
  struct MHD_OptionItem ops[] = {
      {MHD_OPTION_THREAD_POOL_SIZE, _maxClients, nullptr},
      // TODO use a real C++11 HTTP framework
      {MHD_OPTION_NOTIFY_COMPLETED,
       reinterpret_cast<intptr_t>(&requestCompleted),
       static_cast<void *>(this)},
      {MHD_OPTION_END, 0, nullptr}};
  _daemon = MHD_start_daemon(flags, _port, nullptr, nullptr,               //
                             &answerConnection, static_cast<void *>(this), //
                             MHD_OPTION_ARRAY, ops,                        //
                             MHD_OPTION_END);
  if (_daemon == nullptr) {
    return false;
  }

  return true;
}

void HTTPFrontEnd::stop() {
  if (_daemon != nullptr) {
    MHD_stop_daemon(_daemon);
    _daemon = nullptr;
  }
  _numClients = 0;
}

int HTTPFrontEnd::answerConnection(void *cls, struct MHD_Connection *connection,
                                   const char *url, const char *method,
                                   const char *version, const char *uploadData,
                                   size_t *uploadDataSize, void **conCls) {
  if (strcasecmp(method, MHD_HTTP_METHOD_POST) != 0) {
    return respond(connection, none, MHD_HTTP_BAD_REQUEST);
  }

  HTTPFrontEnd *httpServer = static_cast<HTTPFrontEnd *>(cls);
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

    // reserve enough space for an image
    connInfo->imageRaw.reserve(IMAGE_INIT_SIZE);

    connInfo->postProcessor =
        MHD_create_post_processor(connection, POST_BUFFER_SIZE, iteratePost,
                                  static_cast<void *>(connInfo));
    if (connInfo->postProcessor == nullptr) {
      httpServer->_numClients--; // _numClients is atomic
      return MHD_NO;
    }

    *conCls = static_cast<void *>(connInfo);

    return MHD_YES;
  }

  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*conCls);

  if (*uploadDataSize != 0) {
    MHD_post_process(connInfo->postProcessor, uploadData, *uploadDataSize);
    *uploadDataSize = 0;

    return MHD_YES;
  } else {
    std::vector<std::string> results;

    // all data are received
    if (!connInfo->imageRaw.empty()) {
      double fx = connInfo->fx;
      double fy = connInfo->fy;
      double cx = connInfo->cx;
      double cy = connInfo->cy;
      cv::Mat image;
      CameraModel camera;
      createData(connInfo->imageRaw, fx, fy, cx, cy, image, camera);
      if (image.empty()) {
        // TODO do I need to free anything here?
        return respond(connection, none, MHD_HTTP_BAD_REQUEST);
      }

      // blocking wait
      results = httpServer->getOnQuery()(image, camera);
    }

    std::string answer = none;
    if (!results.empty()) {
      answer = std::move(results.at(0));
    }

    return respond(connection, answer, MHD_HTTP_OK);
  }

  return respond(connection, none, MHD_HTTP_BAD_REQUEST);
}

int HTTPFrontEnd::iteratePost(void *coninfo_cls, enum MHD_ValueKind kind,
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
      connInfo->imageRaw.insert(connInfo->imageRaw.end(), data, data + size);
    } else if (strcmp(key, "fx") == 0) {
      // TODO copy to string or vector first
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->fx = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "fy") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->fy = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "cx") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cx = atof(buf);
      delete[] buf;
    } else if (strcmp(key, "cy") == 0) {
      char *buf = new char[size + 1];
      memcpy(buf, data, size);
      buf[size] = 0;
      connInfo->cy = atof(buf);
      delete[] buf;
    }
  }

  return MHD_YES;
}

void HTTPFrontEnd::requestCompleted(void *cls,
                                    struct MHD_Connection *connection,
                                    void **conCls,
                                    enum MHD_RequestTerminationCode toe) {
  HTTPFrontEnd *httpServer = static_cast<HTTPFrontEnd *>(cls);
  assert(httpServer != nullptr);
  ConnectionInfo *connInfo = static_cast<ConnectionInfo *>(*conCls);
  assert(connInfo != nullptr);

  if (connInfo->postProcessor != nullptr) {
    MHD_destroy_post_processor(connInfo->postProcessor);
    httpServer->_numClients--; // _numClients is atomic
  }

  delete connInfo;
  connInfo = nullptr;
  *conCls = nullptr;
}

int HTTPFrontEnd::respond(struct MHD_Connection *connection,
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

void HTTPFrontEnd::createData(const std::vector<char> &data, double fx,
                              double fy, double cx, double cy, cv::Mat &image,
                              CameraModel &camera) {
  // no data copy is needed because conn info
  const bool copyData = false;
  image = imdecode(cv::Mat(data, copyData), cv::IMREAD_GRAYSCALE);

  // imwrite("image.jpg", image);

  int width = image.cols;
  int height = image.rows;
  camera = CameraModel("", fx, fy, cx, cy, cv::Size(width, height));
}
