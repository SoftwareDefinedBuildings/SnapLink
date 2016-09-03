#include <grpc++/grpc++.h>
#include "service/CellMate.grpc.pb.h"

// Class encompasing the state and logic needed to serve a request.
class CallData {
public:
  // Take in the "service" instance (in this case representing an asynchronous
  // server) and the completion queue "cq" used for asynchronous communication
  // with the gRPC runtime.
  CallData(CellMate::AsyncService *service, grpc::ServerCompletionQueue *cq, Feature &feature);

  void proceed();

private:
  // The means of communication with the gRPC runtime for an asynchronous
  // server.
  CellMate::AsyncService *service_;
  // The producer-consumer queue where for asynchronous server notifications.
  grpc::ServerCompletionQueue *cq_;
  // Context for the rpc, allowing to tweak aspects of it such as the use
  // of compression, authentication, as well as to send metadata back to the
  // client.
  grpc::ServerContext ctx_;

  // What we get from the client.
  Query query_;
  // What we send back to the client.
  Empty reply_;

  // The means to get back to the client.
  grpc::ServerAsyncResponseWriter<Empty> responder_;

  // Let's implement a tiny state machine with the following states.
  enum CallStatus { CREATE, PROCESS, FINISH };
  CallStatus status_; // The current serving state.

  Feature& _feature;
};

class CellMateServer final {
public:
  ~CellMateServer();

  // There is no shutdown handling in this code.
  void run();

private:

  // This can be run in multiple threads if needed.
  void handleRpcs();

  std::unique_ptr<ServerCompletionQueue> cq_;
  CellMate::AsyncService service_;
  std::unique_ptr<Server> server_;
  Feature _feature;
};
