#include "run.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/LabelsSimple.h"
#include "lib/data/WordsKdTree.h"
#include "lib/front_end/bosswave/BWFrontEnd.h"
#include "lib/front_end/http/HTTPFrontEnd.h"
#include "run/wrapper/BackEndWrapper.h"
#include "run/wrapper/FrontEndWrapper.h"
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
  std::string bosswaveURI;
  int featureLimit;
  int corrLimit;
  double distRatio;
  std::vector<std::string> dbFiles;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("http,H", po::value<bool>(&http)->default_value(true),
       "run HTTP front end") //
      ("http-port", po::value<int>(&httpPort)->default_value(8080),
       "the port that HTTP front end binds to") //
      ("bosswave,B", po::value<bool>(&bosswave)->default_value(false),
       "run BOSSWAVE front end") //
      ("bosswave-uri", po::value<std::string>(&bosswaveURI)
                           ->default_value("scratch.ns/cellmate"),
       "the URI that BOSSWAVE front end subscribes to") //
      ("feature-limit", po::value<int>(&featureLimit)->default_value(0),
       "limit the number of features used") //
      ("corr-limit", po::value<int>(&corrLimit)->default_value(0),
       "limit the number of corresponding 2D-3D points used") //
      ("dist-ratio", po::value<double>(&distRatio)->default_value(0.7),
       "limit the number of features used");

  po::options_description hidden;
  hidden.add_options() // use comment to force new line using formater
      ("dbfiles",
       po::value<std::vector<std::string>>(&dbFiles)->multitoken()->required(),
       "database files");

  po::options_description all;
  all.add(visible).add(hidden);

  po::positional_options_description pos;
  pos.add("dbfiles", -1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
                                  .options(all)
                                  .positional(pos)
                                  .allow_unregistered()
                                  .run();
  po::store(parsed, vm);

  // print invalid options
  std::vector<std::string> unrecog =
      collect_unrecognized(parsed.options, po::exclude_positional);
  if (unrecog.size() > 0) {
    printInvalid(unrecog);
    printUsage(visible);
    return 1;
  }

  if (vm.count("help")) {
    printUsage(visible);
    return 0;
  }

  // check whether required options exist after handling help
  po::notify(vm);

  // Run the program
  QCoreApplication app(argc, argv);

  std::unique_ptr<WordsKdTree> words(new WordsKdTree());
  std::unique_ptr<LabelsSimple> labels(new LabelsSimple());

  std::unique_ptr<QThread> backEndWrapperThread(new QThread());

  std::cout << "reading data" << std::endl;
  if (!RTABMapAdapter::readData(dbFiles, *words, *labels)) {
    qCritical() << "reading data failed";
    return 1;
  }

  std::cout << "initializing identification service" << std::endl;
  std::shared_ptr<BackEndWrapper> backEndWrapper(new BackEndWrapper(
      std::move(words), std::move(labels), featureLimit, corrLimit, distRatio));
  backEndWrapper->moveToThread(backEndWrapperThread.get());
  backEndWrapperThread->start();

  std::shared_ptr<FrontEndWrapper> httpFrontEndWrapper;
  if (http == true) {
    std::cout << "initializing HTTP front end" << std::endl;
    std::unique_ptr<FrontEnd> httpFrontEnd(
        new HTTPFrontEnd(httpPort, MAX_CLIENTS));
    httpFrontEndWrapper.reset(new FrontEndWrapper(std::move(httpFrontEnd)));
    httpFrontEndWrapper->setBackEndWrapper(backEndWrapper);
    if (!httpFrontEndWrapper->init()) {
      std::cerr << "starting HTTP front end failed";
      return 1;
    }
  }

  std::shared_ptr<FrontEndWrapper> bwFrontEndWrapper;
  if (bosswave == true) {
    std::cerr << "initializing BOSSWAVE front end" << std::endl;
    std::unique_ptr<FrontEnd> bwFrontEnd(new BWFrontEnd(bosswaveURI));
    bwFrontEndWrapper.reset(new FrontEndWrapper(std::move(bwFrontEnd)));
    std::cerr << "DEBUG: bw front end addr" << bwFrontEndWrapper.get()
              << std::endl;
    bwFrontEndWrapper->setBackEndWrapper(backEndWrapper);
    if (!bwFrontEndWrapper->init()) {
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
  std::cout << "cellmate run [command options] db_file..." << std::endl
            << std::endl
            << desc << std::endl;
}
