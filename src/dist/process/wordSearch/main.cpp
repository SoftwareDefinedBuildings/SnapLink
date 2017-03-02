#include "WordSearchServer.h"

int main(int argc, char *argv[]) {
  std::vector<std::string> dbfiles;
  for (int i = 3; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  std::string perspectiveServerAddr(argv[2]); //"0.0.0.0:50053";
  WordSearchServer server;
  if (!server.init(perspectiveServerAddr, dbfiles)) {
    return 1;
  }

  std::string wordSearchServerAddr(argv[1]); // = "0.0.0.0:50052";
  server.run(wordSearchServerAddr);

  return 0;
}
