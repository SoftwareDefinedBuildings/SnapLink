#include "run/run.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/LabelsSimple.h"
#include "lib/data/Transform.h"
#include "lib/data/WordsKdTree.h"
#include "lib/front_end/bosswave/BWFrontEnd.h"
#include "lib/front_end/http/HTTPFrontEnd.h"
#include "lib/util/Utility.h"
#include <QCoreApplication>
#include <cstdio>
#include <utility>

int Run::run(int argc, char *argv[]) {
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
    Run::printInvalid(unrecog);
    Run::printUsage(visible);
    return 1;
  }

  if (vm.count("help")) {
    Run::printUsage(visible);
    return 0;
  }

  // check whether required options exist after handling help
  po::notify(vm);

  // Run the program
  QCoreApplication app(argc, argv);

  std::shared_ptr<Words> words(new WordsKdTree());
  std::unique_ptr<Labels> labels(new LabelsSimple());

  std::cout << "reading data" << std::endl;
  if (!RTABMapAdapter::readData(dbFiles, *words, *labels)) {
    qCritical() << "reading data failed";
    return 1;
  }

  std::cout << "initializing computing stages" << std::endl;
  _feature.reset(new Feature(featureLimit));
  _wordSearch.reset(new WordSearch(words));
  _dbSearch.reset(new DbSearch(words));
  _perspective.reset(new Perspective(words, corrLimit, distRatio));
  _visibility.reset(new Visibility(std::move(labels)));

  std::unique_ptr<FrontEnd> httpFrontEnd;
  if (http == true) {
    std::cout << "initializing HTTP front end" << std::endl;
    httpFrontEnd.reset(new HTTPFrontEnd(httpPort, MAX_CLIENTS));
    if (httpFrontEnd->start() == false) {
      std::cerr << "starting HTTP front end failed";
      return 1;
    }
    httpFrontEnd->registerOnQuery(std::bind(
        &Run::identify, this, std::placeholders::_1, std::placeholders::_2));
  }

  std::unique_ptr<FrontEnd> bwFrontEnd;
  if (bosswave == true) {
    std::cerr << "initializing BOSSWAVE front end" << std::endl;
    bwFrontEnd.reset(new BWFrontEnd(bosswaveURI));
    if (bwFrontEnd->start() == false) {
      std::cerr << "starting BOSSWAVE front end failed";
      return 1;
    }
    bwFrontEnd->registerOnQuery(std::bind(
        &Run::identify, this, std::placeholders::_1, std::placeholders::_2));
  }

  std::cout << "Initialization Done" << std::endl;

  return app.exec();
}

void Run::printInvalid(const std::vector<std::string> &opts) {
  std::cerr << "invalid options: ";
  for (const auto &opt : opts) {
    std::cerr << opt << " ";
  }
  std::cerr << std::endl;
}

void Run::printUsage(const po::options_description &desc) {
  std::cout << "cellmate run [command options] db_file..." << std::endl
            << std::endl
            << desc << std::endl;
}

void Run::printTime(long total, long feature, long wordSearch, long dbSearch,
                    long perspective, long visibility) {
  std::cout << "Time overall: " << total << " ms" << std::endl;
  std::cout << "Time feature: " << feature << " ms" << std::endl;
  std::cout << "Time wordSearch: " << wordSearch << " ms" << std::endl;
  std::cout << "Time dbSearch: " << dbSearch << " ms" << std::endl;
  std::cout << "Time perspective: " << perspective << " ms" << std::endl;
  std::cout << "Time visibility: " << visibility << " ms" << std::endl;
}

std::vector<std::string> Run::identify(const cv::Mat &image,
                                       const CameraModel &camera) {
  std::vector<std::string> results;

  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;

  long startTime;
  long totalStartTime = Utility::getTime();

  startTime = Utility::getTime();
  _feature->extract(image, keyPoints, descriptors);
  long featureTime = Utility::getTime() - startTime;

  // word search
  startTime = Utility::getTime();
  std::vector<int> wordIds = _wordSearch->search(descriptors);
  long wordSearchTime = Utility::getTime() - startTime;

  // db search
  startTime = Utility::getTime();
  int dbId = _dbSearch->search(wordIds);
  long dbSearchTime = Utility::getTime() - startTime;

  // PnP
  Transform pose;
  startTime = Utility::getTime();
  _perspective->localize(wordIds, keyPoints, descriptors, camera, dbId, pose);
  long perspectiveTime = Utility::getTime() - startTime;

  if (pose.isNull()) {
    long totalTime = Utility::getTime() - totalStartTime;
    Run::printTime(totalTime, featureTime, wordSearchTime, dbSearchTime,
                   perspectiveTime, -1);
    return results;
  }

  // visibility
  startTime = Utility::getTime();
  results = _visibility->process(dbId, camera, pose);
  long visibilityTime = Utility::getTime() - startTime;

  long totalTime = Utility::getTime() - totalStartTime;
  Run::printTime(totalTime, featureTime, wordSearchTime, dbSearchTime,
                 perspectiveTime, visibilityTime);

  return results;
}
