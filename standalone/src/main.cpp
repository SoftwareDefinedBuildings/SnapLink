#include "adapter/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/WordsKdTree.h"
#include "front/HTTPServer.h"
#include "front/BWServer.h"
#include "process/Identification.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <utility>

// void showUsage() {
//   printf("\nUsage:\n"
//          "CellMate database_file1 [database_file2 ...]\n");
//   exit(1);
// }

int main(int argc, char *argv[]) {
  // ULogger::setType(ULogger::kTypeConsole);
  // ULogger::setLevel(ULogger::kInfo);
  // ULogger::setLevel(ULogger::kDebug);

  std::vector<std::string> dbfiles;
  for (int i = 1; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }
  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  QThread identThread;

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *labels)) {
    qCritical() << "Reading data failed";
    return 1;
  }

  HTTPServer httpServer;
  BWServer bwServer;
  std::cout << "Initializing Identification Service" << std::endl;
  Identification ident(std::move(words), std::move(labels));
  ident.setHTTPServer(&httpServer);
  ident.setBWServer(&bwServer);
  ident.moveToThread(&identThread);
  identThread.start();
  //BWServer
  std::cout << "Initializing BW server" << std::endl;
  bwServer.setIdentification(&ident);
  QThread bwThread;
  bwThread.start();
  bwServer.moveToThread(&bwThread); 
  QObject::connect(&bwServer, &BWServer::signalBW, &bwServer, &BWServer::startRun);
  emit bwServer.signalBW();
  // HTTPServer
  std::cout << "Initializing HTTP server" << std::endl;
  httpServer.setIdentification(&ident);
  if (!httpServer.start()) {
    qCritical() << "Starting HTTP Server failed";
    return 1;
  }

  return app.exec();
}
