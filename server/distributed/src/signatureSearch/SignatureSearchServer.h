#include "SignatureSearchService.grpc.pb.h"
#include "algo/SignatureSearch.h"
#include <grpc++/grpc++.h>

class SignatureSearchServer final
    : public proto::SignatureSearchService::Service {
public:
  bool init(std::string perspectiveServerAddr,
            std::vector<std::string> dbfiles);
  grpc::Status onWord(grpc::ServerContext *context,
                      const proto::WordMessage *request,
                      proto::Empty *response) override;
  void run(std::string signatureSearchServerAddr);

private:
  std::unique_ptr<SignatureSearch> _signatureSearch;

  std::shared_ptr<grpc::Channel> _channel;
};
