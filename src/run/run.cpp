#include "run.h"
#include "lib/data/LabelsSimple.h"
#include "lib/data/WordsKdTree.h"
#include "lib/adapter/rtabmap/RTABMapDBAdapter.h"
#include "run/front_end/bosswave/BWFrontEndObj.h"
#include "run/front_end/http/HTTPFrontEndObj.h"
#include "run/process/IdentificationObj.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <cstdio>
#include <getopt.h>
#include <utility>

#define MY_OPT 10000

int run(int argc, char *argv[]) {
  bool http;
  bool bosswave;
  int sampleSize;
  int corrSize;
  double distRatio;
  std::vector<std::string> dbfiles;
  try {
    parseOpt(argc, argv, http, bosswave, sampleSize, corrSize, distRatio,
             dbfiles);
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << std::endl;
    showUsage();
    exit(1);
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
  std::shared_ptr<IdentificationObj> identObj(new IdentificationObj(
      std::move(words), std::move(labels), sampleSize, corrSize, distRatio));
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

  QThread bwThread;
  if (bosswave == true) {
    // BWServer
    std::cout << "Initializing BW server" << std::endl;
    unsigned int maxClients = 10;
    std::shared_ptr<BWFrontEndObj> bwFrontEndObj(new BWFrontEndObj());
    identObj->setBWFrontEndObj(bwFrontEndObj);
    bwFrontEndObj->setIdentificationObj(identObj);
    bwThread.start();
    bwFrontEndObj->moveToThread(&bwThread);
    QObject::connect(bwFrontEndObj.get(), &BWFrontEndObj::triggerInit,
                     bwFrontEndObj.get(), &BWFrontEndObj::init);
    emit bwFrontEndObj->triggerInit(maxClients);
  }

  std::cout << "Initialization Done" << std::endl;

  return app.exec();
}

void showUsage() {
  std::cout << "\nUsage:\n"
            << "cellmate [options] [database_file]...\n"
            << "\nOptions:\n"
            << "-h, --help      print this help message\n"
            << "-H, --http      only use HTTP front end\n"
            << "-b, --bosswave  only use BOSSWAVE front end\n"
            << "--sample-size   number of subsamples in feature extraction "
               "(0 means no subsampling, default 200)\n"
            << "--corr-size     number of correspondences in perspective "
               "(default 100)\n"
            << "--dist-ratio    distance ratio used for correspondences "
               "(default 0.7)\n";
}

void parseOpt(int argc, char *argv[], bool &http, bool &bosswave,
              int &sampleSize, int &corrSize, double &distRatio,
              std::vector<std::string> &dbfiles) {

  http = false;
  bosswave = false;
  sampleSize = 200;
  corrSize = 100;
  distRatio = 0.7;

  static struct option longOptions[] = {
      {"help", no_argument, nullptr, 'h'},
      {"http", no_argument, nullptr, 'H'},
      {"bosswave", no_argument, nullptr, 'b'},
      {"sample-size", required_argument, nullptr, MY_OPT},
      {"corr-size", required_argument, nullptr, MY_OPT},
      {"dist-ratio", required_argument, nullptr, MY_OPT},
      {0, 0, 0, 0}};

  int c;
  while (true) {
    int optionIndex = 0;

    c = getopt_long(argc, argv, ":hHb", longOptions, &optionIndex);

    if (c == -1) {
      break;
    }

    switch (c) {
    case 0:
      std::cerr << "invalid option" << std::endl;
      showUsage();
      exit(1);
    case MY_OPT:
      if (strcmp(longOptions[optionIndex].name, "sample-size") == 0) {
        sampleSize = std::stoi(std::string(optarg));
      } else if (strcmp(longOptions[optionIndex].name, "corr-size") == 0) {
        corrSize = std::stoi(std::string(optarg));
      } else if (strcmp(longOptions[optionIndex].name, "dist-ratio") == 0) {
        distRatio = std::stod(std::string(optarg));
      } else {
        std::cerr << "invalid option" << std::endl;
        showUsage();
        exit(1);
      }
      break;
    case 'h':
      showUsage();
      exit(0);
    case 'H':
      http = true;
      break;
    case 'b':
      bosswave = true;
      break;
    case ':': /* missing option argument */
      std::cerr << "missing option argument" << std::endl;
      showUsage();
      exit(1);
    case '?':
      std::cerr << "invalid option" << std::endl;
      showUsage();
      exit(1);
    default:
      showUsage();
      exit(1);
    }
  }

  if (http == false && bosswave == false) {
    http = true;
    bosswave = true;
  }

  for (int i = optind; i < argc; i++) {
    dbfiles.emplace_back(argv[i]);
  }

  std::cout << "http: " << http << std::endl
            << "bosswave: " << bosswave << std::endl
            << "sample-size: " << sampleSize << std::endl
            << "corr-size: " << corrSize << std::endl
            << "dist-ratio: " << distRatio << std::endl;
}
