#pragma once

#include "lib/data/Session.h"
#include <QObject>
#include <QSemaphore>
#include <memory>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <random>

#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000

class CameraModel;

typedef std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera,
             std::unique_ptr<Session> &&session)> OnQueryFunction;

typedef struct {
  struct MHD_PostProcessor *postProcessor;
  std::vector<char> imageRaw;
  double fx;
  double fy;
  double cx;
  double cy;
} ConnectionInfo;

class HTTPServer : public QObject {
public:
  HTTPServer();
  virtual ~HTTPServer();

  bool start(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
  void stop();
  void registerOnQuery(OnQueryFunction onQuery); 

private:
  static int answerConnection(void *cls, struct MHD_Connection *connection,
                              const char *url, const char *method,
                              const char *version, const char *uploadData,
                              size_t *uploadDataSize, void **conCls);
  static int iteratePost(void *coninfo_cls, enum MHD_ValueKind kind,
                         const char *key, const char *filename,
                         const char *contentType,
                         const char *transferEncoding, const char *data,
                         uint64_t off, size_t size);
  static void requestCompleted(void *cls, struct MHD_Connection *connection,
                               void **conCls,
                               enum MHD_RequestTerminationCode toe);
  static int respond(struct MHD_Connection *connection,
                      const std::string &message, int status_code);
  static void createData(const std::vector<char> &data, double fx, double fy,
                         double cx, double cy, cv::Mat &image,
                         CameraModel &camera);

private:
  static const std::string none;

  struct MHD_Daemon *_daemon;
  OnQueryFunction _onQuery; 
  std::atomic<unsigned int> _maxClients;
  std::atomic<unsigned int> _numClients;
  std::mt19937 _gen;
  std::uniform_int_distribution<unsigned long long> _dis;
};
