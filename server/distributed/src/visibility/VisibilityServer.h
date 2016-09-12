#include "VisibilityService.grpc.pb.h"
#include "algo/Visibility.h"
#include <grpc++/grpc++.h>

class VisibilityServer final : public proto::VisibilityService::Service {
public:
  bool init(std::string frontServerAddr, std::vector<std::string> dbfiles);
  grpc::Status onLocation(grpc::ServerContext *context,
                          const proto::LocationMessage *request,
                          proto::Empty *response) override;
  void run(std::string visibilityServerAddr);

private:
  std::unique_ptr<Visibility> _visibility;

  std::shared_ptr<grpc::Channel> _channel;
};
