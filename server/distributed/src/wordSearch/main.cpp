#include "WordSearchServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  std::string signatureSearchServerAddr = "0.0.0.0:50053";
  WordSearchServer server;
  if (!server.init(signatureSearchServerAddr, dbfiles)) {
    return 1;
  }

  std::string wordSearchServerAddr = "0.0.0.0:50052";
  server.run(wordSearchServerAddr);

  return 0;
}
