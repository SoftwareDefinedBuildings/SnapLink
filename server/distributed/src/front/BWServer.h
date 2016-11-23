#pragma once

#include "FrontService.grpc.pb.h"
#include "data/Session.h"
#include <QSemaphore>
#include <grpc++/grpc++.h>
#include <memory>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <random>

#define PORT 8080
#define MAX_CLIENTS 10
#define POST_BUFFER_SIZE 100000
#define IMAGE_INIT_SIZE 100000

class CameraModel;

enum ConnectionType { POST = 0 };

// TODO delete after combine cameranetwork and http server
typedef struct {
  double fx;
  double fy;
  double cx;
  double cy;
} CameraInfo;

typedef struct {
  enum ConnectionType sessionType;
  struct MHD_PostProcessor *postProcessor;
  CameraInfo cameraInfo;
  QSemaphore detected;
  std::string answerString;
  std::unique_ptr<std::vector<std::string>> names;
  std::unique_ptr<Session> session;
  std::unique_ptr<std::vector<char>> rawData;
} ConnectionInfo;

class BWServer final : public proto::FrontService::Service {
public:
  BWServer();
  ~BWServer();

  bool init(std::string featureServerAddr, uint16_t port = PORT,
            unsigned int maxClients = MAX_CLIENTS);
  grpc::Status onDetection(grpc::ServerContext *context,
                           const proto::DetectionMessage *request,
                           proto::Empty *response) override;
  void run(std::string bwServerAddr);

  int getMaxClients() const;
  int getNumClients() const;
  void setNumClients(int numClients);

private:
  // HTTPServer handler
  static int answerConnection(void *cls, struct MHD_Connection *connection,
                              const char *url, const char *method,
                              const char *version, const char *upload_data,
                              size_t *upload_data_size, void **con_cls);
  static int iteratePost(void *coninfo_cls, enum MHD_ValueKind kind,
                         const char *key, const char *filename,
                         const char *content_type,
                         const char *transfer_encoding, const char *data,
                         uint64_t off, size_t size);
  static void requestCompleted(void *cls, struct MHD_Connection *connection,
                               void **con_cls,
                               enum MHD_RequestTerminationCode toe);
  static int sendPage(struct MHD_Connection *connection,
                      const std::string &page, int status_code);
  static void createData(const std::vector<char> &data, double fx, double fy,
                         double cx, double cy, cv::Mat &image,
                         CameraModel &camera);

private:
  static const std::string busypage;
  static const std::string completepage;
  static const std::string errorpage;
  static const std::string servererrorpage;

  struct MHD_Daemon *_daemon;
  unsigned int _maxClients;
  unsigned int _numClients;
  std::mutex _mutex;
  std::mt19937 _gen;
  std::uniform_int_distribution<unsigned long long> _dis;
  std::map<long, ConnectionInfo *> _connInfoMap;

  std::shared_ptr<grpc::Channel> _channel;
};
