#include "PerspectiveService.grpc.pb.h"
#include "algo/Perspective.h"
#include <grpc++/grpc++.h>

class PerspectiveServer final : public proto::PerspectiveService::Service {
public:
  bool init(std::string visibilityServerAddr, std::vector<std::string> dbfiles);
  grpc::Status onWord(grpc::ServerContext *context,
                      const proto::WordMessage *request,
                      proto::Empty *response) override;
  void run(std::string perspectiveServerAddr);

private:
  std::unique_ptr<Perspective> _perspective;

  std::shared_ptr<grpc::Channel> _channel;
};
