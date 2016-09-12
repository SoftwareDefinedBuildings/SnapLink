#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/SignaturesSimple.h"
#include "data/WordsKdTree.h"
#include "visibility/VisibilityServer.h"
#include <cstdio>
#include <grpc++/grpc++.h>
#include <utility>

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  VisibilityServer server;
  if (!server.init(dbfiles)) {
    return 1;
  }

  server.run();

  return 0;
}
