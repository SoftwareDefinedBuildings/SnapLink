#include "http/HTTPServer.h"
#include <QDebug>

int main() {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  if (!httpServer.init()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  httpServer.run();

  return 0;
}
