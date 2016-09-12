#include "PerspectiveServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  PerspectiveServer server;
  std::string visibilityServerAddr = "0.0.0.0:50055";
  if (!server.init(visibilityServerAddr, dbfiles)) {
    return 1;
  }

  std::string perspectiveServerAddr = "0.0.0.0:50054";
  server.run(perspectiveServerAddr);

  return 0;
}
