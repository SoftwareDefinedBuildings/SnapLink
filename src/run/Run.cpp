#include "run/Run.h"
#include "lib/adapter/rtabmap/RTABMapAdapter.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Label.h"
#include "lib/data/Transform.h"
#include "lib/front_end/grpc/GrpcFrontEnd.h"
#include "lib/util/Utility.h"
#include <QCoreApplication>
#include <cstdio>
#include <utility>
#include <zbar.h> 
#include <pthread.h>
#include <QtConcurrent>

int Run::run(int argc, char *argv[]) {
  // Parse arguments
  int port;
  int featureLimit;
  int corrLimit;
  float distRatio;
  std::vector<std::string> dbFiles;

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("port", po::value<int>(&port)->default_value(8080),
       "the port that GRPC front end binds to") //
      ("feature-limit", po::value<int>(&featureLimit)->default_value(0),
       "limit the number of features used") //
      ("corr-limit", po::value<int>(&corrLimit)->default_value(0),
       "limit the number of corresponding 2D-3D points used") //
      ("dist-ratio", po::value<float>(&distRatio)->default_value(0.7),
       "distance ratio used to create words");

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

  std::cout << "READING DATABASES" << std::endl;
  RTABMapAdapter adapter(distRatio);
  if (!adapter.init(std::set<std::string>(dbFiles.begin(), dbFiles.end()))) {
    std::cerr << "reading data failed";
    return 1;
  }

  const std::map<int, Word> &words = adapter.getWords();
  const std::map<int, Room> &rooms = adapter.getRooms();
  const std::map<int, std::vector<Label>> &labels = adapter.getLabels();

  std::cout << "RUNNING COMPUTING ELEMENTS" << std::endl;
  _feature = std::make_unique<Feature>(featureLimit);
  _wordSearch = std::make_unique<WordSearch>(words);
  _roomSearch = std::make_unique<RoomSearch>(rooms, words);
  _perspective =
      std::make_unique<Perspective>(rooms, words, corrLimit, distRatio);
  _visibility = std::make_unique<Visibility>(labels);

  std::unique_ptr<FrontEnd> frontEnd;
  std::cerr << "initializing GRPC front end" << std::endl;
  frontEnd = std::make_unique<GrpcFrontEnd>(port, MAX_CLIENTS);
  if(frontEnd->start() == false) {
    std::cerr << "starting GRPC front end failed";
    return 1;
  }
  frontEnd->registerOnQuery(std::bind(
      &Run::identify, this, std::placeholders::_1, std::placeholders::_2));
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

void Run::printTime(long total, long feature, long wordSearch, long roomSearch,
                    long perspective, long visibility) {
  std::cout << "Time overall: " << total << " ms" << std::endl;
  std::cout << "Time feature: " << feature << " ms" << std::endl;
  std::cout << "Time wordSearch: " << wordSearch << " ms" << std::endl;
  std::cout << "Time roomSearch: " << roomSearch << " ms" << std::endl;
  std::cout << "Time perspective: " << perspective << " ms" << std::endl;
  std::cout << "Time visibility: " << visibility << " ms" << std::endl;
}

// must be thread safe
std::vector<FoundItem> Run::identify(const cv::Mat &image,
                                       const CameraModel &camera) {
  std::vector<FoundItem> results;
  std::vector<FoundItem> qrResults;
  long startTime;
  long totalStartTime = Utility::getTime();

  // qr extraction
  QFuture<bool> qrWatcher =  QtConcurrent::run(qrExtract, image, &qrResults);
  
  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  long featureTime;
  {
    std::lock_guard<std::mutex> lock(_featureMutex);
    startTime = Utility::getTime();
    _feature->extract(image, keyPoints, descriptors);
    featureTime = Utility::getTime() - startTime;
  }

  // word search
  std::vector<int> wordIds;
  long wordSearchTime;
  {
    std::lock_guard<std::mutex> lock(_wordSearchMutex);
    startTime = Utility::getTime();
    wordIds = _wordSearch->search(descriptors);
    wordSearchTime = Utility::getTime() - startTime;
  }

  // room search
  int roomId;
  long roomSearchTime;
  {
    std::lock_guard<std::mutex> lock(_roomSearchMutex);
    startTime = Utility::getTime();
    roomId = _roomSearch->search(wordIds);
    roomSearchTime = Utility::getTime() - startTime;
  }

  // PnP
  Transform pose;
  long perspectiveTime;
  {
    std::lock_guard<std::mutex> lock(_perspectiveMutex);
    startTime = Utility::getTime();
    pose =
        _perspective->localize(wordIds, keyPoints, descriptors, camera, roomId);
    perspectiveTime = Utility::getTime() - startTime;
  }

  if (pose.isNull()) {
    std::cerr << "image localization failed (did you provide the correct "
                 "intrinsic matrix?)"
              << std::endl;
    long totalTime = Utility::getTime() - totalStartTime;
    Run::printTime(totalTime, featureTime, wordSearchTime, roomSearchTime,
                   perspectiveTime, -1);
  } else {
     // visibility
    long visibilityTime;
    {
      std::lock_guard<std::mutex> lock(_visibilityMutex);
      startTime = Utility::getTime();
      results = _visibility->process(roomId, camera, pose);
      visibilityTime = Utility::getTime() - startTime;
    }

    long totalTime = Utility::getTime() - totalStartTime;

    // std::cout << "image pose :" << std::endl << pose << std::endl;
    Run::printTime(totalTime, featureTime, wordSearchTime, roomSearchTime,
                   perspectiveTime, visibilityTime);
  }

  qrWatcher.waitForFinished();
  if(qrWatcher.result()) {
    results.insert(results.begin(), qrResults.begin(), qrResults.end());
  }

  return results;
}

bool Run::qrExtract(const cv::Mat &im, std::vector<FoundItem> *results) {
  std::cout<<"Qr extracting\n";
  zbar::ImageScanner scanner;   
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);   
  uchar *raw = (uchar *)im.data;
  int width = im.cols;
  int height = im.rows;
  std::cout<<"Widith = "<<width<<" Height = "<<height<<std::endl;
  zbar::Image image(width, height, "Y800", raw, width * height);
  int n = scanner.scan(image); 
  std::cout<<"n is "<<n<<std::endl;
  for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  symbol != image.symbol_end(); ++symbol) {
    std::cout << "decoded " << symbol->get_type_name() << " symbol "<< symbol->get_data() << std::endl; 
    double x0 = symbol->get_location_x(0);
    double x1 = symbol->get_location_x(1);
    double x2 = symbol->get_location_x(2);
    double x3 = symbol->get_location_x(3);
    double y0 = symbol->get_location_y(0);
    double y1 = symbol->get_location_y(1);
    double y2 = symbol->get_location_y(2);
    double y3 = symbol->get_location_y(3);
    double meanX = (x0 + x1 + x2 + x3)/4;
    double meanY = (y0 + y1 + y2 + y3)/4;
    double width = (meanX - x0) > 0 ? (meanX - x0) : (x0 - meanX); 
    FoundItem item(symbol->get_data(), meanX, meanY, width);
    std::cout<<"Width is "<<width<<std::endl;
    std::cout<<"X is "<<meanX<<std::endl;
    std::cout<<"Y is "<<meanY<<std::endl;
    results->push_back(item);
  } 
  
  return results->size() > 0;
}
