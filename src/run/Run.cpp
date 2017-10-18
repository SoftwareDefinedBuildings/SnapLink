#include "run/Run.h"
#include "lib/data/FoundItem.h"
#include "lib/data/Label.h"
#include "lib/data/Transform.h"
#include "lib/front_end/grpc/GrpcFrontEnd.h"
#include "lib/util/Utility.h"
#include "lib/visualize/visualize.h"
#include <QCoreApplication>
#include <QtConcurrent>
#include <cstdio>
#include <pthread.h>
#include <utility>

int Run::run(int argc, char *argv[]) {
  // Parse arguments
  po::options_description visible("command options");
  visible.add_options() // use comment to force new line using formater
      ("help,h", "print help message") //
      ("port,p", po::value<int>(&_port)->default_value(8080),
       "the port that GRPC front end binds to") //
      ("feature-limit,f", po::value<int>(&_featureLimit)->default_value(0),
       "limit the number of features used") //
      ("visualize,v", po::value<int>(&_visCount)->default_value(0),
       "Show localized camera pose in 3D model up to n latest poses") //
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
  std::cout << "READING DATABASES" << std::endl;
  _adapter = std::make_unique<RTABMapAdapter>(_distRatio);
  if (!_adapter->init(std::set<std::string>(_dbFiles.begin(), _dbFiles.end()))) {
    std::cerr << "reading data failed";
    return 1;
  }

  words = _adapter->getWords();
  rooms = _adapter->getRooms();
  labels = _adapter->getLabels();

  if (_visCount > 0) {
    _visualize =
        std::make_unique<Visualize>(_adapter->getImages(), _visCount);
    QtConcurrent::run(_visualize.get(), &Visualize::startVis);
  }

  std::cout << "RUNNING COMPUTING ELEMENTS" << std::endl;
  _feature = std::make_unique<Feature>(_featureLimit);
  _wordSearch = std::make_unique<WordSearch>(words);
  _roomSearch = std::make_unique<RoomSearch>(rooms, words);
  _perspective =
      std::make_unique<Perspective>(rooms, words, _corrLimit, _distRatio);
  _visibility = std::make_unique<Visibility>(labels);
  _aprilTag = std::make_unique<Apriltag>(_tagSize);
  _QR = std::make_unique<QR>();

  std::cerr << "initializing GRPC front end" << std::endl;
  std::unique_ptr<FrontEnd> frontEnd = std::make_unique<GrpcFrontEnd>(_port, MAX_CLIENTS);
  if (frontEnd->start() == false) {
    std::cerr << "starting GRPC front end failed";
    return 1;
  }
  frontEnd->registerLocalizeFunc(
      std::bind(&Run::localize, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));
  frontEnd->registerGetLabelsFunc(std::bind(&Run::getLabels, this));
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
  std::cout << "snaplink run [command options] db_file..." << std::endl
            << std::endl
            << desc << std::endl;
}

// must be thread safe
std::pair<int, Transform> Run::localize(const cv::Mat &image,
                                        const CameraModel &camera,
                                        std::vector<FoundItem> *items) {
  std::cout << "***New Query Image***" << std::endl;
  std::vector<FoundItem> qrResults;
  int dbId;
  Transform imgPose;
  long startTime;
  long totalStartTime = Utility::getTime();

  std::pair<std::vector<int>, std::vector<Transform>> aprilDetectResult;
  std::pair<int, Transform> imageLocResultPose;

  QFuture<std::pair<std::vector<int>, std::vector<Transform>>> aprilDetectWatcher;
  QFuture<std::pair<int, Transform>> imageLocalizeWatcher;

  // qr extraction
  QFuture<std::vector<FoundItem>> qrWatcher =
      QtConcurrent::run(_QR.get(), &QR::QRdetect, image);

  if (_saveImage) {
    QtConcurrent::run([=]() { imwrite(std::to_string(Utility::getTime()) + ".jpg", image); });
  }

  if (!camera.isValid()) {
    std::cerr << "Warning: Camera is invalid." << std::endl;
  } else {
    // aprilTag extraction and localization
    aprilDetectWatcher = QtConcurrent::run(_aprilTag.get(), &Apriltag::aprilDetect, image, camera);
    imageLocalizeWatcher = QtConcurrent::run(this, &Run::imageLocalize, image, camera);

    aprilDetectResult = aprilDetectWatcher.result();
    // This lookup and localize part takes less than 1ms, so didn't put in another thread to run
    std::vector<Transform> tagPoseInCamFrame = aprilDetectResult.second;
    std::vector<std::pair<int, Transform>> tagPoseInModelFrame = _adapter->lookupAprilCodes(aprilDetectResult.first);
    std::vector<std::pair<int, Transform>> aprilResultPose = _aprilTag->aprilLocalize(tagPoseInCamFrame, tagPoseInModelFrame);

    imageLocResultPose = imageLocalizeWatcher.result();

    // Select the final pose to use in vis from multiple pose candidates
    // Prioritize the pose derived from April Tag 
    if (aprilResultPose.size() > 0) {
      // TODO multiple april tags
      dbId = aprilResultPose[0].first;
      imgPose = aprilResultPose[0].second;
    } else {
      dbId = imageLocResultPose.first;
      imgPose = imageLocResultPose.second;
    }

    if (_visCount > 0) {
      _visualize->setPose(dbId, imgPose, image, camera);
    }

    if (imgPose.isNull() == false && items != nullptr) {
      // visibility
      {
        std::lock_guard<std::mutex> lock(_visibilityMutex);
        startTime = Utility::getTime();
        *items = _visibility->process(dbId, camera, imgPose);
        long visibilityTime = Utility::getTime() - startTime;
        std::cout << "Time visibility " << visibilityTime << " ms" << std::endl;
      }
    }
  }

  qrResults = qrWatcher.result();
  if (items != nullptr) {
    items->insert(items->end(), qrResults.begin(), qrResults.end());
  }

  long totalTime = Utility::getTime() - totalStartTime;
  std::cout << "Time Localization overall " << totalTime << " ms" << std::endl;

  if (aprilDetectResult.first.size() > 0 && !imageLocResultPose.second.isNull()) {
    QtConcurrent::run(this, &Run::calculateAndSaveAprilTagPose,
                      aprilDetectResult.second, aprilDetectResult.first,
                      imageLocResultPose);
  }

  return std::make_pair(dbId, imgPose);
}

std::pair<int, Transform> Run::imageLocalize(const cv::Mat &image,
                                             const CameraModel &camera) {
  // feature extraction
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;
  long featureTime;
  {
    std::lock_guard<std::mutex> lock(_featureMutex);
    long startTime = Utility::getTime();
    _feature->extract(image, keyPoints, descriptors);
    featureTime = Utility::getTime() - startTime;
  }

  // word search
  std::vector<int> wordIds;
  long wordSearchTime;
  {
    std::lock_guard<std::mutex> lock(_wordSearchMutex);
    long startTime = Utility::getTime();
    wordIds = _wordSearch->search(descriptors);
    wordSearchTime = Utility::getTime() - startTime;
  }

  // room search
  int dbId;
  long roomSearchTime;
  {
    std::lock_guard<std::mutex> lock(_roomSearchMutex);
    long startTime = Utility::getTime();
    dbId = _roomSearch->search(wordIds);
    roomSearchTime = Utility::getTime() - startTime;
  }

  // PnP
  Transform pose;
  long perspectiveTime;
  {
    std::lock_guard<std::mutex> lock(_perspectiveMutex);
    long startTime = Utility::getTime();
    pose =
        _perspective->localize(wordIds, keyPoints, descriptors, camera, dbId);
    perspectiveTime = Utility::getTime() - startTime;
  }

  std::cout << "Time feature: " << featureTime << " ms" << std::endl;
  std::cout << "Time wordSearch: " << wordSearchTime << " ms" << std::endl;
  std::cout << "Time roomSearch: " << roomSearchTime << " ms" << std::endl;
  std::cout << "Time perspective: " << perspectiveTime << " ms" << std::endl;

  return std::make_pair(dbId, pose);
}

void Run::calculateAndSaveAprilTagPose(
    std::vector<Transform> aprilTagPosesInCamFrame,
    std::vector<int> aprilTagCodes,
    std::pair<int, Transform> imageLocResultPose) {

  for (unsigned int i = 0; i < aprilTagPosesInCamFrame.size(); i++) {
    Transform tagPoseInCamFrame = aprilTagPosesInCamFrame[i];
    int code = aprilTagCodes[i];

    int dbId = imageLocResultPose.first;
    Transform camPoseInModelFrame = imageLocResultPose.second;
    Transform tagPoseInModelFrame =
        _aprilTag->calculateNewAprilTagPoseInModelFrame(camPoseInModelFrame,
                                                        tagPoseInCamFrame);
    long time = Utility::getTime();
    _adapter->saveAprilTagPose(dbId, time, code, tagPoseInModelFrame, 1);
  }
}

std::map<int, std::vector<Label>> Run::getLabels() {
  return _adapter->getLabels();
}
