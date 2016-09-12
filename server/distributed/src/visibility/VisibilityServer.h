#include "VisibilityService.grpc.pb.h"
#include "algo/Perspective.h"
#include "algo/Visibility.h"
#include <grpc++/grpc++.h>

class VisibilityServer final : public proto::VisibilityService::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onLocation(grpc::ServerContext *context,
                          const proto::LocationMessage *request,
                          proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Visibility> _visibility;

  std::shared_ptr<grpc::Channel> _channel;
};
