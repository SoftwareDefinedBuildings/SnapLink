#include "adapter/rtabmap/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/WordsKdTree.h"
#include "front_end/bosswave/BWServer.h"
#include "front_end/http/HTTPFrontEndObj.h"
#include "process/IdentificationObj.h"
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

  QThread identObjThread;

  std::cout << "Reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbfiles, *words, *labels)) {
    qCritical() << "Reading data failed";
    return 1;
  }

  std::cout << "Initializing IdentificationObj Service" << std::endl;
  std::shared_ptr<IdentificationObj> identObj(
      new IdentificationObj(std::move(words), std::move(labels)));
  identObj->moveToThread(&identObjThread);
  identObjThread.start();

  // BWServer
  std::cout << "Initializing BW server" << std::endl;
  // TODO use shared_ptr
  std::shared_ptr<BWFrontEndObj> bwFrontEndObj(new BWFrontEndObj());
  identObj->setBWFrontEndObj(bwFrontEndObj);
  bwFrontEndObj->setIdentificationObj(identObj);
  QThread bwThread;
  bwThread.start();
  bwFrontEndObj->moveToThread(&bwThread);
  QObject::connect(bwFrontEndObj, &BWFrontEndObj::triggerInit, bwFrontEndObj,
                   &BWFrontEndObj::init);
  emit bwFrontEndObj->triggerInit();

  // HTTPFrontEndObj
  std::cout << "Initializing HTTP Front End" << std::endl;
  std::shared_ptr<HTTPFrontEndObj> httpFrontEndObj(new HTTPFrontEndObj());
  identObj->setHTTPFrontEndObj(httpFrontEndObj);
  httpFrontEndObj->setIdentificationObj(identObj);
  if (!httpFrontEndObj->init()) {
    qCritical() << "Starting HTTP Front End Failed";
    return 1;
  }

  std::cout << "Initialization Done" << std::endl;

  return app.exec();
}
