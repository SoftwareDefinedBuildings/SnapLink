#pragma once

#include "lib/algo/Feature.h"
#include "lib/algo/Perspective.h"
#include "lib/algo/RoomSearch.h"
#include "lib/algo/Visibility.h"
#include "lib/algo/WordSearch.h"
#include "lib/algo/Apriltag.h"
#include "lib/algo/QR.h"
#include <boost/program_options.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include "lib/visualize/visualize.h"
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

  // must be thread-safe
  // camera is optional, no image localization is performed if not provided
  std::pair<int, Transform> localize(const cv::Mat &image, const CameraModel &camera, std::vector<FoundItem> *items); 
  std::map<int, std::vector<Label>> getLabels();
  bool qrExtract(const cv::Mat &image, std::vector<FoundItem> *results);
  
  void calculateAndSaveAprilTagPose(std::vector<Transform> aprilTagPosesInCamFrame,std::vector<int> aprilTagCodes,std::pair<int, Transform> imageLocResultPose);  

  std::vector<std::pair<int, Transform>> aprilLocalize(const cv::Mat &im, const CameraModel &camera, double tagSize,std::vector<Transform> *tagPoseInCamFrame, std::vector<int> *tagCodes);

  std::pair<int, Transform> imageLocalize(const cv::Mat &image, const CameraModel &camera);

private:
  int _port;
  int _featureLimit;
  int _corrLimit;
  float _distRatio;
  std::vector<std::string> _dbFiles;
  bool _saveImage;
  int _visCount;
  double _tagSize;
  std::unique_ptr<RTABMapAdapter> _adapter;
  std::unique_ptr<Visualize> _visualize;

  std::unique_ptr<Feature> _feature;
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<RoomSearch> _roomSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;
  std::unique_ptr<Apriltag> _aprilTag;
  std::unique_ptr<QR> _QR;

  std::mutex _featureMutex;
  std::mutex _wordSearchMutex;
  std::mutex _roomSearchMutex;
  std::mutex _perspectiveMutex;
  std::mutex _visibilityMutex;
};
