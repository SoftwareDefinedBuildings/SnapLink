#include "SignatureSearchServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  SignatureSearchServer server;
  std::string perspectiveServerAddr = "0.0.0.0:50054";
  if (!server.init(perspectiveServerAddr, dbfiles)) {
    return 1;
  }

  std::string signatureSearchServerAddr = "0.0.0.0:50053";
  server.run(signatureSearchServerAddr);

  return 0;
}
