#include "BWFrontEndServer.h"
#include <QDebug>
#include <QThread>

int main(int argc, char *argv[]) {
  QCoreApplication a(argc, argv);
  std::cout << "Initializing BW FrontEnd server" << std::endl;
  BWFrontEndServer bwFrontEndServer;
  std::string featureServerAddr(argv[2]); // = "0.0.0.0:50051";
  if (!bwFrontEndServer.init(featureServerAddr)) {
    qCritical() << "Starting BW Server failed";
    return 1;
  }

  std::string bwFrontEndServerAddr(argv[1]); // = "0.0.0.0:50056";
  qDebug() << "lalala1";
  QThread grpcThread;
  grpcThread.start();
  bwFrontEndServer.moveToThread(&grpcThread);
  QObject::connect(&bwFrontEndServer, &BWFrontEndServer::triggerRun,
                   &bwFrontEndServer, &BWFrontEndServer::run);
  emit bwFrontEndServer.triggerRun(
      QString::fromStdString(bwFrontEndServerAddr));
  // bwFrontEndServer.run(bwFrontEndServerAddr);
  qDebug() << "lalala";
  return a.exec();
}
