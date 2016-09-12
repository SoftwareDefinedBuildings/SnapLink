#include "HTTPServer.h"
#include <QDebug>

int main() {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  std::string featureServerAddr = "0.0.0.0:50051";
  if (!httpServer.init(featureServerAddr)) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  std::string httpServerAddr = "0.0.0.0:50056";
  httpServer.run(httpServerAddr);

  return 0;
}
