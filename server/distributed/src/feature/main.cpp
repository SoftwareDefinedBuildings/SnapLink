#include "FeatureServer.h"

int main(int argc, char *argv[]) {
  FeatureServer server;
  std::string wordSeachServerAddr(argv[2]); // = "0.0.0.0:50052";
  if (!server.init(wordSeachServerAddr)) {
    return 1;
  }

  std::string featureServerAddr(argv[1]); // = "0.0.0.0:50051";
  server.run(featureServerAddr);

  return 0;
}
