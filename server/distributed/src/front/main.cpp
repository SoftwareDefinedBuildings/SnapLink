#include "HTTPServer.h"
#include <QDebug>

int main() {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  if (!httpServer.init("0.0.0.0:50056")) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  httpServer.run("0.0.0.0:50051");

  return 0;
}
