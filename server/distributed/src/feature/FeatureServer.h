#include "FeatureService.grpc.pb.h"
#include "algo/Feature.h"
#include <grpc++/grpc++.h>

class FeatureServer final : public proto::FeatureService::Service {
public:
  bool init(std::string wordSearchServerAddr);
  grpc::Status onQuery(grpc::ServerContext *context,
                       const proto::QueryMessage *request,
                       proto::Empty *response) override;
  void run(std::string featureServerAddr);

private:
  std::unique_ptr<Feature> _feature;

  std::shared_ptr<grpc::Channel> _channel;
};
