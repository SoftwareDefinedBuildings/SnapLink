#include "VisibilityServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  VisibilityServer server;
  std::string frontServerAddr = "0.0.0.0:50056";
  if (!server.init(frontServerAddr, dbfiles)) {
    return 1;
  }

  std::string visibilityServerAddr = "0.0.0.0:50055";
  server.run(visibilityServerAddr);

  return 0;
}
