#include "CellMate.grpc.pb.h"
#include "algo/Feature.h"
#include <grpc++/grpc++.h>

class FeatureServer final : public proto::Feature::Service {
public:
  bool init();
  grpc::Status onQuery(grpc::ServerContext *context,
                       const proto::Query *request,
                       proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Feature> _feature;

  std::shared_ptr<grpc::Channel> _channel;
};
