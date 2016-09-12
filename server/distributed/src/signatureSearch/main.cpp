#include "SignatureSearchServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  SignatureSearchServer server;
  if (!server.init(dbfiles)) {
    return 1;
  }

  server.run();

  return 0;
}
