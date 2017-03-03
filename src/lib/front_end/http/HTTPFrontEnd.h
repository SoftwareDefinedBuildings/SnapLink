#pragma once

#include "lib/front_end/FrontEnd.h"
#include <atomic>
#include <microhttpd.h>
#include <mutex>

#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000

typedef struct final {
  struct MHD_PostProcessor *postProcessor;
  std::vector<char> imageRaw;
  double fx;
  double fy;
  double cx;
  double cy;
} ConnectionInfo;

class HTTPFrontEnd final : public FrontEnd
{
public:
  explicit HTTPFrontEnd(uint16_t port, unsigned int maxClients);
  ~HTTPFrontEnd();

  bool start() final;
  void stop() final;

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
  uint16_t _port;
  struct MHD_Daemon *_daemon;
  std::mutex _mutex; // to protect _numClient
  std::atomic<unsigned int> _numClients;
  std::atomic<unsigned int> _maxClients;
};
