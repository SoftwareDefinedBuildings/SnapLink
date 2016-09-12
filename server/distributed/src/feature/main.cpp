#include "FeatureServer.h"

int main() {
  FeatureServer server;
  std::string wordSeachServerAddr = "0.0.0.0:50052";
  if (!server.init(wordSeachServerAddr)) {
    return 1;
  }

  std::string featureServerAddr = "0.0.0.0:50051";
  server.run(featureServerAddr);

  return 0;
}
