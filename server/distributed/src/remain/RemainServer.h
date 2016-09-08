#include "RemainService.grpc.pb.h"
#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/SignatureSearch.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <grpc++/grpc++.h>

class RemainServer final : public proto::RemainService::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onFeature(grpc::ServerContext *context,
                         const proto::FeatureMessage *request,
                         proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<SignatureSearch> _signatureSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;

  std::shared_ptr<grpc::Channel> _channel;
};
