#include "HTTPServer.h"
#include <QDebug>

int main(int argc, char *argv[]) {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  std::string featureServerAddr(argv[2]); // = "0.0.0.0:50051";
  if (!httpServer.init(featureServerAddr)) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  std::string httpServerAddr(argv[1]); // = "0.0.0.0:50056";
  httpServer.run(httpServerAddr);

  return 0;
}
