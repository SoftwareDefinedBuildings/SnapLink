#include "WordSearchService.grpc.pb.h"
#include "algo/WordSearch.h"
#include <grpc++/grpc++.h>

class WordSearchServer final : public proto::WordSearchService::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onFeature(grpc::ServerContext *context,
                         const proto::FeatureMessage *request,
                         proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<WordSearch> _wordSearch;

  std::shared_ptr<grpc::Channel> _channel;
};
