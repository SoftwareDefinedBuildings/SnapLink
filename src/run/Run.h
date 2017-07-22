#pragma once

#include "lib/algo/Feature.h"
#include "lib/algo/Perspective.h"
#include "lib/algo/RoomSearch.h"
#include "lib/algo/Visibility.h"
#include "lib/algo/WordSearch.h"
#include <boost/program_options.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include "lib/adapter/rtabmap/RTABMapAdapter.h"

#define MAX_CLIENTS 10

namespace po = boost::program_options;

class CameraModel;
class FoundItem;
class Run final {
public:
  int run(int argc, char *argv[]);

private:
  static void printInvalid(const std::vector<std::string> &opts);
  static void printUsage(const po::options_description &desc);
  static void printTime(long total, long feature, long wordSearch,
                        long roomSearch, long perspective, long visibility);
  // must be thread-safe
  // camera is optional, no image localization is performed if not provided
  std::vector<FoundItem> identify(const cv::Mat &image,
                                  const CameraModel *camera);
  static bool qrExtract(const cv::Mat &image, std::vector<FoundItem> *results);
  static bool aprilExtract(const cv::Mat &im, const CameraModel *camera,
         std::vector<Transform> &poses, double tagSize, std::vector<int> &codesOfTags); 
  
  void calculateAndSaveAprilTagPose(std::vector<Transform> &tagPosesInCameraFrame, Transform &pose, std::vector<int> codes, int roomId);

  enum Mode{QR_ONLY, FULL_FUNCTIONING};

private:
  int _port;
  int _featureLimit;
  int _corrLimit;
  float _distRatio;
  std::vector<std::string> _dbFiles;
  bool _saveImage;
  double _tagSize;
  RTABMapAdapter _adapter;

  std::unique_ptr<Feature> _feature;
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<RoomSearch> _roomSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;

  std::mutex _featureMutex;
  std::mutex _wordSearchMutex;
  std::mutex _roomSearchMutex;
  std::mutex _perspectiveMutex;
  std::mutex _visibilityMutex;
  Run::Mode _mode;
};
