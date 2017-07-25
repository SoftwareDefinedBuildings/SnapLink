#include "run/Run.h"
#include "apriltags/CameraUtil.h"
#include "apriltags/TagDetector.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Label.h"
#include "lib/data/Transform.h"
#include "lib/front_end/grpc/GrpcFrontEnd.h"
#include "lib/util/Utility.h"
#include <QCoreApplication>
#include <QtConcurrent>
#include <cstdio>
#include <pthread.h>
#include <utility>
#include <zbar.h>
int Run::run(int argc, char *argv[]) {
  // Parse arguments

  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("port,p", po::value<int>(&_port)->default_value(8080),
       "the port that GRPC front end binds to") //
      ("feature-limit,f", po::value<int>(&_featureLimit)->default_value(0),
       "limit the number of features used") //
      ("corr-limit,c", po::value<int>(&_corrLimit)->default_value(0),
       "limit the number of corresponding 2D-3D points used") //
      ("save-image,s", po::bool_switch(&_saveImage)->default_value(false),
       "save images to files, which can causes significant delays.") //
      ("tag-size, z", po::value<double>(&_tagSize)->default_value(0.16),
       "size of april-tags used in the room") //
      ("dist-ratio,d", po::value<float>(&_distRatio)->default_value(0.7),
       "distance ratio used to create words");

  po::options_description hidden;
  hidden.add_options() // use comment to force new line using formater
      ("dbfiles", po::value<std::vector<std::string>>(&_dbFiles)
                      ->multitoken()
                      ->default_value(std::vector<std::string>(), ""),
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
  std::map<int, Word> words;
  std::map<int, Room> rooms;
  std::map<int, std::vector<Label>> labels;
  if (_dbFiles.size()) {
    _mode = Run::FULL_FUNCTIONING;
    std::cout << "READING DATABASES" << std::endl;
    _adapter = RTABMapAdapter(_distRatio);
    if (!_adapter.init(
            std::set<std::string>(_dbFiles.begin(), _dbFiles.end()))) {
      std::cerr << "reading data failed";
      return 1;
    }

    words = _adapter.getWords();
    rooms = _adapter.getRooms();
    labels = _adapter.getLabels();

    std::cout << "RUNNING COMPUTING ELEMENTS" << std::endl;
    _feature = std::make_unique<Feature>(_featureLimit);
    _wordSearch = std::make_unique<WordSearch>(words);
    _roomSearch = std::make_unique<RoomSearch>(rooms, words);
    _perspective =
        std::make_unique<Perspective>(rooms, words, _corrLimit, _distRatio);
    _visibility = std::make_unique<Visibility>(labels);
  } else {
    _mode = Run::QR_ONLY;
    std::cout << "Only QR identification is enabled" << std::endl;
  }

  std::unique_ptr<FrontEnd> frontEnd;
  std::cerr << "initializing GRPC front end" << std::endl;
  frontEnd = std::make_unique<GrpcFrontEnd>(_port, MAX_CLIENTS);
  if (frontEnd->start() == false) {
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

  std::vector<Transform> aprilTagPoses;
  std::vector<int> aprilTagCodes;

  std::vector<std::pair<int, Transform>> aprilResultPose;
  std::pair<int, Transform> imageLocResultPose;

  // qr extraction
  QFuture<bool> qrWatcher =
      QtConcurrent::run(this, &Run::qrExtract, image, &qrResults);

  if (_saveImage) {
    QtConcurrent::run(
        [=]() { imwrite(std::to_string(Utility::getTime()) + ".jpg", image); });
  }

  if (_mode == Run::FULL_FUNCTIONING && camera.isValid()) {
    // aprilTag extraction and localization
    QFuture<std::vector<std::pair<int, Transform>>> aprilWatcher =
        QtConcurrent::run(this, &Run::aprilExtract, image, camera, _tagSize,
                          &aprilTagPoses, &aprilTagCodes);

    QFuture<std::pair<int, Transform>> imageLocalizeWatcher =
        QtConcurrent::run(this, &Run::imageLocalize, image, camera);

    aprilResultPose = aprilWatcher.result();
    imageLocResultPose = imageLocalizeWatcher.result();

    // Select the final pose to use in vis from multiple pose candidates
    Transform finalPose = aprilResultPose[0].second;
    int roomId = aprilResultPose[0].first;

    if (finalPose.isNull()) {
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
        results = _visibility->process(roomId, camera, finalPose);
        visibilityTime = Utility::getTime() - startTime;
      }

      long totalTime = Utility::getTime() - totalStartTime;

      // std::cout << "image pose :" << std::endl << pose << std::endl;
      Run::printTime(totalTime, featureTime, wordSearchTime, roomSearchTime,
                     perspectiveTime, visibilityTime);
    }
  } else {
    std::cerr << "image localization not performed." << std::endl;
  }

  if (qrWatcher.result()) {
    results.insert(results.begin(), qrResults.begin(), qrResults.end());
  }

  if (aprilWatcher.result().size() > 0) {
    QtConcurrent::run(this, &Run::calculateAndSaveAprilTagPose, aprilTagPoses,
                      aprilTagCodes, imageLocResultPose);
  }

  return results;
}

std::pair<int, Transform> Run::imageLocalize(const cv::Mat &image,
                                             const CameraModel &camera) {
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

  return std::make_pair(roomId, pose);
}

bool Run::qrExtract(const cv::Mat &im, std::vector<FoundItem> *results) {
  std::cout << "Qr extracting\n";
  zbar::ImageScanner scanner;
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  uchar *raw = (uchar *)im.data;
  int width = im.cols;
  int height = im.rows;
  std::cout << "Widith = " << width << " Height = " << height << std::endl;
  zbar::Image image(width, height, "Y800", raw, width * height);

  int n = scanner.scan(image);
  std::cout << "n is " << n << std::endl;
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
       symbol != image.symbol_end(); ++symbol) {
    std::cout << "decoded " << symbol->get_type_name() << " symbol "
              << symbol->get_data() << std::endl;
    // Zbar's coordinate system is topleft = (0,0)
    // x axis is pointing to right
    // y axis is pointing to bottom

    double x0 = symbol->get_location_x(0);
    double x1 = symbol->get_location_x(1);
    double x2 = symbol->get_location_x(2);
    double x3 = symbol->get_location_x(3);
    double y0 = symbol->get_location_y(0);
    double y1 = symbol->get_location_y(1);
    double y2 = symbol->get_location_y(2);
    double y3 = symbol->get_location_y(3);
    double meanX = (x0 + x1 + x2 + x3) / 4;
    double meanY = (y0 + y1 + y2 + y3) / 4;
    double size = (meanX - x0) > 0 ? (meanX - x0) : (x0 - meanX);
    FoundItem item(symbol->get_data(), meanX, meanY, size, width, height);
    std::cout << "Size is " << size << std::endl;
    std::cout << "X is " << meanX << std::endl;
    std::cout << "Y is " << meanY << std::endl;
    results->push_back(item);
  }

  return results->size() > 0;
}

std::vector<std::pair<int, Transform>> Run::aprilExtract(
    const cv::Mat &im, const CameraModel &camera, double tagSize,
    std::vector<Transform> *tagPoseInCamFrame std::vector<int> *tagCodes) {
  std::vector<std::pair<int, Transform>> results;

  TagDetectorParams params;
  TagFamily family("Tag36h11");
  TagDetector detector(family, params);
  TagDetectionArray detections;

  cv::Point2d opticalCenter;
  opticalCenter.x = camera.cx();
  opticalCenter.y = camera.cy();

  detector.process(im, opticalCenter, detections);

  cv::Mat r, t;
  for (unsigned int i = 0; i < detections.size(); i++) {
    CameraUtil::homographyToPoseCV(camera.fx(), camera.fy(), tagSize,
                                   detections[i].homography, r, t);
    Transform pose(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2),
                   t.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(1, 1),
                   r.at<double>(1, 2), t.at<double>(0, 1), r.at<double>(2, 0),
                   r.at<double>(2, 1), r.at<double>(2, 2), t.at<double>(0, 2));
    tagPoseInCamFrame->push_back(pose);
    tagCodes->push_back(detections[i].code);

    std::pair<int, Transform> tagPoseInModelFrame =
        _adapter.lookupAprilCode(detections[i].code);
    results.push_back(
        std::make_pair(tagPoseInModelFrame.first,
                       tagPoseInModelFrame.second * pose.inverse()));
  }

  return results;
}

void Run::calculateAndSaveAprilTagPose(
    std::vector<Transform> aprilTagPosesInCamFrame,
    std::vector<int> aprilTagCodes,
    std::pair<int, Transform> imageLocResultPose) {

  for (unsigned int i = 0; i < aprilTagPosesInCamFrame.size(); i++) {
    Transform tagPoseInCamFrame = aprilTagPosesInCamFrame[i];
    int code = aprilTagCodes[i];

    int roomId = imageLocResultPose.first;
    Transform camPoseInModelFrame = imageLocResultPose.second;
    Transform tagPoseInModelFrame = camPoseInModelFrame * tagPoseInCamFrame;
    long time = Utility::getTime();
    _adapter.saveAprilTagPose(roomId, time, code, tagPoseInModelFrame);
  }
}
