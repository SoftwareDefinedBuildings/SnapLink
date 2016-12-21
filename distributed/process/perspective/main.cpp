#include "PerspectiveServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 3; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  PerspectiveServer server;
  std::string visibilityServerAddr(argv[2]); // = "0.0.0.0:50055";
  if (!server.init(visibilityServerAddr, dbfiles)) {
    return 1;
  }

  std::string perspectiveServerAddr(argv[1]); // = "0.0.0.0:50054";
  server.run(perspectiveServerAddr);

  return 0;
}
