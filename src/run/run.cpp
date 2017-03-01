#include "run.h"
#include "lib/adapter/rtabmap/RTABMapDBAdapter.h"
#include "lib/data/LabelsSimple.h"
#include "lib/data/WordsKdTree.h"
#include "run/front_end/bosswave/BWFrontEndObj.h"
#include "run/front_end/http/HTTPFrontEndObj.h"
#include "run/process/IdentificationObj.h"
#include <QCoreApplication>
#include <QDebug>
#include <QThread>
#include <boost/program_options.hpp>
#include <cstdio>
#include <getopt.h>
#include <utility>

namespace po = boost::program_options;

static void printInvalid(const std::vector<std::string> &opts);
static void printUsage(const po::options_description &desc);

int run(int argc, char *argv[]) {
  // Parse arguments
  bool http;
  int httpPort;
  bool bosswave;
  std::string bosswaveTopic;
  int featureLimit;
  int corrLimit;
  double distRatio;
  std::vector<std::string> dbFiles;

  po::options_description run("command options");
  run.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("http,H", po::value<bool>(&http)->default_value(true),
       "run HTTP front end") //
      ("http-port", po::value<int>(&httpPort)->default_value(8080),
       "the port that HTTP front end binds to") //
      ("bosswave,B", po::value<bool>(&bosswave)->default_value(false),
       "run BOSSWAVE front end") //
      ("bosswave-topic", po::value<std::string>(&bosswaveTopic)
                             ->default_value("scratch.ns/cellmate"),
       "the topic that BOSSWAVE front end subscribes to") //
      ("feature-limit", po::value<int>(&featureLimit)->default_value(0),
       "limit the number of features used") //
      ("corr-limit", po::value<int>(&corrLimit)->default_value(0),
       "limit the number of corresponding 2D-3D points used") //
      ("dist-ratio", po::value<double>(&distRatio)->default_value(0.7),
       "limit the number of features used") //
      ("dbfiles", po::value<std::vector<std::string>>(&dbFiles)->multitoken(),
       "database files");

  po::positional_options_description pos;
  pos.add("dbfiles", -1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
                                  .options(run)
                                  .positional(pos)
                                  .allow_unregistered()
                                  .run();
  po::store(parsed, vm);
  po::notify(vm);

  // print invalid options
  std::vector<std::string> unrecog =
      collect_unrecognized(parsed.options, po::exclude_positional);
  if (unrecog.size() > 0) {
    printInvalid(unrecog);
    printUsage(run);
    return 1;
  }

  if (vm.count("help")) {
    printUsage(run);
    return 0;
  }

  // Run the program
  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  QThread identObjThread;

  std::cout << "reading data" << std::endl;
  if (!RTABMapDBAdapter::readData(dbFiles, *words, *labels)) {
    qCritical() << "reading data failed";
    return 1;
  }

  std::cout << "initializing identification service" << std::endl;
  std::shared_ptr<IdentificationObj> identObj(new IdentificationObj(
      std::move(words), std::move(labels), featureLimit, corrLimit, distRatio));
  identObj->moveToThread(&identObjThread);
  identObjThread.start();

  if (http == true) {
    // HTTPFrontEndObj
    std::cout << "initializing HTTP front end" << std::endl;
    std::shared_ptr<HTTPFrontEndObj> httpFrontEndObj(new HTTPFrontEndObj());
    identObj->setHTTPFrontEndObj(httpFrontEndObj);
    httpFrontEndObj->setIdentificationObj(identObj);
    if (!httpFrontEndObj->init(httpPort)) {
      std::cerr << "starting HTTP front end failed";
      return 1;
    }
  }

  if (bosswave == true) {
    // BWServer
    std::cerr << "initializing BOSSWAVE front end" << std::endl;
    std::shared_ptr<BWFrontEndObj> bwFrontEndObj(new BWFrontEndObj());
    identObj->setBWFrontEndObj(bwFrontEndObj);
    bwFrontEndObj->setIdentificationObj(identObj);
    if (!bwFrontEndObj->init(bosswaveTopic)) {
      std::cerr << "starting BOSSWAVE front end failed";
      return 1;
    }
  }

  std::cout << "Initialization Done" << std::endl;

  return app.exec();
}

static void printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

static void printUsage(const po::options_description &desc) {
  std::cout << "cellmate run [command options]" << std::endl
            << std::endl
            << desc << std::endl;
}
