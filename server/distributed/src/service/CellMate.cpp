#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "front/HTTPServer.h"
#include "service/FeatureStage.h"
#include "service/PerspectiveStage.h"
#include "service/SignatureSearchStage.h"
#include "service/VisibilityStage.h"
#include "service/WordSearchStage.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <utility>
#include <grpc++/grpc++.h>

#include "service/CellMate.grpc.pb.h"

class CellMateImpl final {
 public:
  ~CellMateImpl() {
    server_->Shutdown();
    // Always shutdown the completion queue after the server.
    cq_->Shutdown();
  }

  // There is no shutdown handling in this code.
  void Run() {
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
    HandleRpcs();
  }

 private:
  // Class encompasing the state and logic needed to serve a request.
  class CallData {
   public:
    // Take in the "service" instance (in this case representing an asynchronous
    // server) and the completion queue "cq" used for asynchronous communication
    // with the gRPC runtime.
    CallData(CellMate::AsyncService* service, grpc::ServerCompletionQueue* cq)
        : service_(service), cq_(cq), responder_(&ctx_), status_(CREATE) {
      // Invoke the serving logic right away.
      Proceed();
    }

    void Proceed() {
      if (status_ == CREATE) {
        // Make this instance progress to the PROCESS state.
        status_ = PROCESS;

        // As part of the initial CREATE state, we *request* that the system
        // start processing SayHello requests. In this request, "this" acts are
        // the tag uniquely identifying the request (so that different CallData
        // instances can serve different requests concurrently), in this case
        // the memory address of this CallData instance.
        service_->RequestDetect(&ctx_, &request_, &responder_, cq_, cq_,
                                  this);
      } else if (status_ == PROCESS) {
        // Spawn a new CallData instance to serve new clients while we process
        // the one for this CallData. The instance will deallocate itself as
        // part of its FINISH state.
        new CallData(service_, cq_);

        // The actual processing.
        // TODO

        // And we are done! Let the gRPC runtime know we've finished, using the
        // memory address of this instance as the uniquely identifying tag for
        // the event.
        status_ = FINISH;
        responder_.Finish(reply_, Status::OK, this);
      } else {
        GPR_ASSERT(status_ == FINISH);
        // Once in the FINISH state, deallocate ourselves (CallData).
        delete this;
      }
    }

   private:
    // The means of communication with the gRPC runtime for an asynchronous
    // server.
    CellMate::AsyncService* service_;
    // The producer-consumer queue where for asynchronous server notifications.
    grpc::ServerCompletionQueue* cq_;
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
    CallStatus status_;  // The current serving state.
  };

  // This can be run in multiple threads if needed.
  void HandleRpcs() {
    // Spawn a new CallData instance to serve new clients.
    new CallData(&service_, cq_.get());
    void* tag;  // uniquely identifies a request.
    bool ok;
    while (true) {
      // Block waiting to read the next event from the completion queue. The
      // event is uniquely identified by its tag, which in this case is the
      // memory address of a CallData instance.
      // The return value of Next should always be checked. This return value
      // tells us whether there is any kind of event or cq_ is shutting down.
      GPR_ASSERT(cq_->Next(&tag, &ok));
      GPR_ASSERT(ok);
      static_cast<CallData*>(tag)->Proceed();
    }
  }

  std::unique_ptr<ServerCompletionQueue> cq_;
  CellMate::AsyncService service_;
  std::unique_ptr<Server> server_;
};


int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::shared_ptr<SignaturesSimple> signatures(new SignaturesSimple());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  QThread featureThread;
  QThread wordSearchThread;
  QThread signatureSearchThread;
  QThread perspectiveThread;
  QThread visThread;

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *signatures, *labels)) {
    qCritical() << "Reading data failed";
    return 1;
  }

  HTTPServer httpServer;
  FeatureStage feature;
  WordSearchStage wordSearch(std::move(words));
  SignatureSearchStage signatureSearch(signatures);
  PerspectiveStage perspective(signatures);
  VisibilityStage vis(std::move(labels));

  // VisibilityStage
  std::cout << "Initializing VisibilityStage" << std::endl;
  vis.setHTTPServer(&httpServer);
  vis.moveToThread(&visThread);
  visThread.start();

  // PerspectiveStage
  std::cout << "Initializing PerspectiveStage" << std::endl;
  perspective.setHTTPServer(&httpServer);
  perspective.setVisibilityStage(&vis);
  perspective.moveToThread(&perspectiveThread);
  perspectiveThread.start();

  // Signature Search
  std::cout << "Initializing Signature Search" << std::endl;
  signatureSearch.setPerspectiveStage(&perspective);
  signatureSearch.moveToThread(&signatureSearchThread);
  signatureSearchThread.start();

  // Word Search
  std::cout << "Initializing Word Search" << std::endl;
  wordSearch.setSignatureSearchStage(&signatureSearch);
  wordSearch.moveToThread(&wordSearchThread);
  wordSearchThread.start();

  // FeatureStage
  std::cout << "Initializing feature extraction" << std::endl;
  feature.setWordSearchStage(&wordSearch);
  feature.moveToThread(&featureThread);
  featureThread.start();

  // HTTPServer
  std::cout << "Initializing HTTP server" << std::endl;
  httpServer.setFeatureStage(&feature);
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  return app.exec();
}
