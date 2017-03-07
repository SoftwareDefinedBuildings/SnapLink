#pragma once

#include "lib/algo/Feature.h"
#include "lib/algo/WordSearch.h"
#include "lib/algo/DbSearch.h"
#include "lib/algo/Perspective.h"
#include "lib/algo/Visibility.h"
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <memory>

#define MAX_CLIENTS 10

namespace po = boost::program_options;

class CameraModel;

class Run final {
public:
  int run(int argc, char *argv[]);

private:
  static void printInvalid(const std::vector<std::string> &opts);
  static void printUsage(const po::options_description &desc);
  static void printTime(long total, long feature, long wordSearch, long dbSearch, long perspective, long visibility);
  std::vector<std::string> identify(const cv::Mat &image, const CameraModel &camera);

private:
  std::unique_ptr<Feature> _feature;
  std::unique_ptr<WordSearch> _wordSearch;
  std::unique_ptr<DbSearch> _dbSearch;
  std::unique_ptr<Perspective> _perspective;
  std::unique_ptr<Visibility> _visibility;
};
