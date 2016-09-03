#include "front/HTTPServer.h"

int main(int argc, char **argv) {
  std::cout << "Initializing HTTP server" << std::endl;
  httpServer.setFeatureStage(&feature);
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  return 0;
}
