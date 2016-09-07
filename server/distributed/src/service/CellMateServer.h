#include "CellMate.grpc.pb.h"
#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/SignatureSearch.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <grpc++/grpc++.h>

// Class encompasing the state and logic needed to serve a request.
class CallData {
public:
  // Take in the "service" instance (in this case representing an asynchronous
  // server) and the completion queue "cq" used for asynchronous communication
  // with the gRPC runtime.
  CallData(proto::CellMate::AsyncService *service,
           grpc::ServerCompletionQueue *cq, Feature &feature,
           WordSearch &wordSearch, SignatureSearch &signatureSearch,
           Perspective &perspective, Visibility &visibility);

  void proceed();

private:
  // The means of communication with the gRPC runtime for an asynchronous
  // server.
  proto::CellMate::AsyncService *service_;
  // The producer-consumer queue where for asynchronous server notifications.
  grpc::ServerCompletionQueue *cq_;
  // Context for the rpc, allowing to tweak aspects of it such as the use
  // of compression, authentication, as well as to send metadata back to the
  // client.
  grpc::ServerContext ctx_;

  // What we get from the client.
  proto::Query query_;
  // What we send back to the client.
  proto::Empty reply_;

  // The means to get back to the client.
  grpc::ServerAsyncResponseWriter<proto::Empty> responder_;

  // Let's implement a tiny state machine with the following states.
  enum CallStatus { CREATE, PROCESS, FINISH };
  CallStatus status_; // The current serving state.

  Feature &_feature;
  WordSearch &_wordSearch;
  SignatureSearch &_signatureSearch;
  Perspective &_perspective;
  Visibility &_visibility;
};

class CellMateServer final {
public:
  ~CellMateServer();

  bool init(std::vector<std::string> dbfiles);
  // There is no shutdown handling in this code.
  void run();

private:
  // This can be run in multiple threads if needed.
  void handleRpcs();

  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  proto::CellMate::AsyncService service_;
  std::unique_ptr<grpc::Server> server_;
  std::unique_ptr<Feature> _feature;
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<SignatureSearch> _signatureSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;
};
