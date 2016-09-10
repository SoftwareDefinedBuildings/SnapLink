#include "RemainService.grpc.pb.h"
#include "algo/Perspective.h"
#include "algo/Visibility.h"
#include <grpc++/grpc++.h>

class RemainServer final : public proto::RemainService::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onSignature(grpc::ServerContext *context,
                           const proto::SignatureMessage *request,
                           proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;

  std::shared_ptr<grpc::Channel> _channel;
};
