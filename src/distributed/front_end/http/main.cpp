#include "HTTPFrontEndServer.h"
#include <QDebug>

int main(int argc, char *argv[]) {
  std::cout << "Initializing HTTP FrontEnd server" << std::endl;
  HTTPFrontEndServer httpFrontEndServer;
  std::string featureServerAddr(argv[2]); // = "0.0.0.0:50051";
  if (!httpFrontEndServer.init(featureServerAddr)) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  std::string httpFrontEndServerAddr(argv[1]); // = "0.0.0.0:50056";
  httpFrontEndServer.run(httpFrontEndServerAddr);

  return 0;
}
