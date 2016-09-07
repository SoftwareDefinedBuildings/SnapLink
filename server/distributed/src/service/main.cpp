#include "CellMate.grpc.pb.h"
#include "CellMateServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "front/HTTPServer.h"
#include "service/FeatureStage.h"
#include "service/PerspectiveStage.h"
#include "service/SignatureSearchStage.h"
#include "service/VisibilityStage.h"
#include "service/WordSearchStage.h"
#include <cstdio>
#include <grpc++/grpc++.h>
#include <utility>

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  CellMateServer server;
  if (!server.init(dbfiles)) {
    return 1;
  }

  server.run();

  return 0;
}
