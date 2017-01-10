#include "adapter/rtabmap/RTABMapDBAdapter.h"
#include "data/LabelsSimple.h"
#include "data/WordsKdTree.h"
#include "front_end/bosswave/BWFrontEndObj.h"
#include "front_end/http/HTTPFrontEndObj.h"
#include "process/IdentificationObj.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <getopt.h>
#include <utility>

void showUsage() {
  printf("\nUsage:\n"
         "CellMate [options] database_file1 [database_file2 ...]\n"
         "\nOptions:\n"
         "-h only use HTTP front end\n"
         "-b only use BOSSWAVE front end\n");
}

void parseOpt(int argc, char *argv[], bool &http, bool &bosswave,
              std::vector<std::string> &dbfiles) {

  http = false;
  bosswave = false;

  static struct option long_options[] = {
      {"http", no_argument, nullptr, 'h'},
      {"bosswave", no_argument, nullptr, 'b'},
      {0, 0, 0, 0}};

  int c;
  while (true) {
    int option_index = 0;

    c = getopt_long(argc, argv, "h::b::", long_options, &option_index);

    if (c == -1) {
      break;
    }

    switch (c) {
    case 'h':
      http = true;
      break;
    case 'b':
      bosswave = true;
      break;
    default:
      showUsage();
      abort();
    }
  }

  if (http == false && bosswave == false) {
    http = true;
    bosswave = true;
  }

  for (int i = optind; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }
}

int main(int argc, char *argv[]) {
  // ULogger::setType(ULogger::kTypeConsole);
  // ULogger::setLevel(ULogger::kInfo);
  // ULogger::setLevel(ULogger::kDebug);

  bool http;
  bool bosswave;
  std::vector<std::string> dbfiles;
  parseOpt(argc, argv, http, bosswave, dbfiles);

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

  if (http == true) {
    // HTTPFrontEndObj
    std::cout << "Initializing HTTP Front End" << std::endl;
    std::shared_ptr<HTTPFrontEndObj> httpFrontEndObj(new HTTPFrontEndObj());
    identObj->setHTTPFrontEndObj(httpFrontEndObj);
    httpFrontEndObj->setIdentificationObj(identObj);
    if (!httpFrontEndObj->init()) {
      qCritical() << "Starting HTTP Front End Failed";
      return 1;
    }
  }

  if (bosswave == true) {
    // BWServer
    std::cout << "Initializing BW server" << std::endl;
    // TODO use shared_ptr
    unsigned int maxClients = 10;
    std::shared_ptr<BWFrontEndObj> bwFrontEndObj(new BWFrontEndObj());
    identObj->setBWFrontEndObj(bwFrontEndObj);
    bwFrontEndObj->setIdentificationObj(identObj);
    QThread bwThread;
    bwThread.start();
    bwFrontEndObj->moveToThread(&bwThread);
    QObject::connect(bwFrontEndObj.get(), &BWFrontEndObj::triggerInit,
                     bwFrontEndObj.get(), &BWFrontEndObj::init);
    emit bwFrontEndObj->triggerInit(maxClients);
  }

  std::cout << "Initialization Done" << std::endl;

  return app.exec();
}
