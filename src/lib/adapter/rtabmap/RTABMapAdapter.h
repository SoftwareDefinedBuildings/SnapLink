#pragma once

#include "lib/data/Label.h"
#include "lib/data/Word.h"
#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <pcl/filters/extract_indices.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/VisualWord.h>
#include <set>
#include <typeinfo>

class Labels;
class Words;

class RTABMapAdapter final {
public:
  // read data from database files
  static bool readData(const std::vector<std::string> &dbPaths, Words &words,
                       Labels &labels);

private:
  static std::map<int, std::unique_ptr<rtabmap::Signature>>
  readSignatures(const std::string &dbPath);
  static std::list<std::unique_ptr<Label>> readLabels(
      const std::string &dbPath, int dbId,
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);

  static std::map<int, rtabmap::Transform>
  getOptimizedPoseMap(const std::string &dbPath);

  static std::list<std::unique_ptr<Word>> createWords(
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);
  static std::list<std::unique_ptr<Word>>
  clusterPointsInWords(std::list<std::unique_ptr<Word>> &words);

  static bool getPoint3World(const rtabmap::Signature &signature,
                             const cv::Point2f &point2, pcl::PointXYZ &point3);
  static std::vector<pcl::PointXYZ>
  clusterPoints3(const std::vector<pcl::PointXYZ> &points3,
                 std::vector<pcl::PointIndices> *clusterIndicesOut = nullptr);
};
