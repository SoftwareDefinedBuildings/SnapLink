#pragma once

#include <memory>
#include <atomic>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/core.hpp>

#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000

class CameraModel;

typedef struct {
  struct MHD_PostProcessor *postProcessor;
  std::vector<char> imageRaw;
  double fx;
  double fy;
  double cx;
  double cy;
} ConnectionInfo;

class HTTPFrontEnd {
public:
  HTTPFrontEnd();
  virtual ~HTTPFrontEnd();

  bool start(uint16_t port, unsigned int maxClients);
  void stop();
  void registerOnQuery(std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> onQuery);

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
  std::function<std::vector<std::string>(std::unique_ptr<cv::Mat> &&image,
             std::unique_ptr<CameraModel> &&camera)> _onQuery; 
  std::mutex _mutex;
  std::atomic<unsigned int> _maxClients;
  std::atomic<unsigned int> _numClients;
};
