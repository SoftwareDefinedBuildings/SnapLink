#include "http/HTTPServer.h"
#include <QDebug>

int main() {
  std::cout << "Initializing HTTP server" << std::endl;
  HTTPServer httpServer;
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  (void)getchar();

  return 0;
}
