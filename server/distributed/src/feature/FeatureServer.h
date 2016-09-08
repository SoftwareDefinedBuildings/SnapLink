#include "FeatureService.grpc.pb.h"
#include "algo/Feature.h"
#include <grpc++/grpc++.h>

class FeatureServer final : public proto::FeatureService::Service {
public:
  bool init();
  grpc::Status onQuery(grpc::ServerContext *context,
                       const proto::QueryMessage *request,
                       proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Feature> _feature;

  std::shared_ptr<grpc::Channel> _channel;
};
