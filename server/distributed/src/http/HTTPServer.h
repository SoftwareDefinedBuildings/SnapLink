#pragma once

#include "Http.grpc.pb.h"
#include "data/Session.h"
#include <QSemaphore>
#include <grpc++/grpc++.h>
#include <grpc++/grpc++.h>
#include <memory>
#include <microhttpd.h>
#include <mutex>
#include <opencv2/core/core.hpp>
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

class HTTPServer final {
public:
  HTTPServer();
  ~HTTPServer();

  bool startMHD(uint16_t port = PORT, unsigned int maxClients = MAX_CLIENTS);
  void runGRPC();

  int getMaxClients() const;
  int getNumClients() const;
  void setNumClients(int numClients);

private:
  // Class encompasing the state and logic needed to serve a request.
  class CallData {
  public:
    // Take in the "service" instance (in this case representing an asynchronous
    // server) and the completion queue "cq" used for asynchronous communication
    // with the gRPC runtime.
    CallData(proto::Http::AsyncService *service,
             grpc::ServerCompletionQueue *cq,
             std::map<long, ConnectionInfo *> &connInfoMap);

    void proceed();

  private:
    // The means of communication with the gRPC runtime for an asynchronous
    // server.
    proto::Http::AsyncService *service_;
    // The producer-consumer queue where for asynchronous server notifications.
    grpc::ServerCompletionQueue *cq_;
    // Context for the rpc, allowing to tweak aspects of it such as the use
    // of compression, authentication, as well as to send metadata back to the
    // client.
    grpc::ServerContext ctx_;

    // What we get from the client.
    proto::Detection _detection;
    // What we send back to the client.
    proto::Empty reply_;

    // The means to get back to the client.
    grpc::ServerAsyncResponseWriter<proto::Empty> responder_;

    // Let's implement a tiny state machine with the following states.
    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_; // The current serving state.

    std::map<long, ConnectionInfo *> &_connInfoMap;
  };

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

  // GRPC HTTPServer handler
  // This can be run in multiple threads if needed.
  void handleRpcs();

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
  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  proto::Http::AsyncService service_;
  std::unique_ptr<grpc::Server> server_;
};
