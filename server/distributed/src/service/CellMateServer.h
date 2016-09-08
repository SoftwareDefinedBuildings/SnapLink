#include "CellMate.grpc.pb.h"
#include "algo/Feature.h"
#include "algo/Perspective.h"
#include "algo/SignatureSearch.h"
#include "algo/Visibility.h"
#include "algo/WordSearch.h"
#include <grpc++/grpc++.h>

class CellMateServer final : public proto::CellMate::Service {
public:
  bool init(std::vector<std::string> dbfiles);
  grpc::Status onQuery(grpc::ServerContext *context,
                       const proto::Query *request,
                       proto::Empty *response) override;
  void run();

private:
  std::unique_ptr<Feature> _feature;
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<SignatureSearch> _signatureSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;

  std::shared_ptr<grpc::Channel> _channel;
};
