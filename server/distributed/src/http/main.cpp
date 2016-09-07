#include "http/HTTPServer.h"
#include <QDebug>

int main() {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  if (!httpServer.startMHD()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  httpServer.runGRPC();

  return 0;
}
