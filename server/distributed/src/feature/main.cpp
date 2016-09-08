#include "FeatureServer.h"
#include "adapter/RTABMapDBAdapter.h"
#include <cstdio>
#include <grpc++/grpc++.h>
#include <utility>

int main() {
  FeatureServer server;
  if (!server.init()) {
    return 1;
  }

  server.run();

  return 0;
}
