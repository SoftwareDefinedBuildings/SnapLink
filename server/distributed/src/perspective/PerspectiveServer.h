#include "PerspectiveService.grpc.pb.h"
#include "algo/Perspective.h"
#include "algo/Visibility.h"
#include <grpc++/grpc++.h>

class PerspectiveServer final : public proto::PerspectiveService::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onSignature(grpc::ServerContext *context,
                           const proto::SignatureMessage *request,
                           proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Perspective> _perspective;

  std::shared_ptr<grpc::Channel> _channel;
};
