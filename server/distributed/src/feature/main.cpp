#include "FeatureServer.h"

int main() {
  FeatureServer server;
  if (!server.init()) {
    return 1;
  }

  server.run();

  return 0;
}
