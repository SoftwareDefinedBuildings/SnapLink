#pragma once

#include "lib/data/Label.h"
#include "lib/data/Word.h"
#include "lib/data/Room.h"
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

class Words;

class RTABMapAdapter final {
public:
  // read data from database files
  static bool readData(const std::vector<std::string> &dbPaths, std::map<int, Word> &words, std::map<int, Room> &rooms,
                       std::map<int, std::list<Label>> &labels);

private:
  static std::map<int, std::unique_ptr<rtabmap::Signature>>
  readSignatures(const std::string &dbPath);
  static std::list<Label> readDBLabels(
      const std::string &dbPath, int dbId,
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);

  static std::map<int, rtabmap::Transform>
  getOptimizedPoseMap(const std::string &dbPath);

  static std::map<int, Word> createWords(
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);
  static std::map<int, Room> createRooms(const std::map<int, Word> &words);

  static bool getPoint3World(const rtabmap::Signature &signature,
                             const cv::Point2f &point2, pcl::PointXYZ &point3);
};
