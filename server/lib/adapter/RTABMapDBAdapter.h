#pragma once

#include "data/Label.h"
#include "data/Signature.h"
#include "data/Word.h"
#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Transform.h>
#include <set>
#include <typeinfo>

class Signatures;
class Labels;
class Words;

class RTABMapDBAdapter {
public:
  /**
   * read data from database files, NULL pointers will be ignored
   */
  static bool readData(const std::vector<std::string> &dbPaths, Words &words,
                       Signatures &signatures, Labels &labels);

private:
  static std::map<int, std::unique_ptr<rtabmap::Signature>>
  readSignatures(const std::string &dbPath);
  static std::list<std::unique_ptr<Word>> readWords(
      const std::string &dbPath, int dbId,
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &signatures);
  static std::list<std::unique_ptr<Label>> readLabels(
      const std::string &dbPath, int dbId,
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);

  static std::map<int, rtabmap::Transform>
  getOptimizedPoseMap(const std::string &dbPath);

  static bool getPoint3World(
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures,
      int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);

  static std::map<std::pair<int, int>, int> getMergeWordsIdMap(
      const std::map<int, std::list<std::unique_ptr<Word>>> &allWords);
  static std::map<std::pair<int, int>, int> getMergeSignaturesIdMap(
      const std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &allSignatures);
  static std::list<std::unique_ptr<Word>>
  mergeWords(std::map<int, std::list<std::unique_ptr<Word>>> &&allWords,
             const std::map<std::pair<int, int>, int> &mergeWordsIdMap,
             const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap);
  static std::list<std::unique_ptr<Signature>> mergeSignatures(
      std::map<int, std::map<int, std::unique_ptr<rtabmap::Signature>>>
          &&allSignatures,
      const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap,
      const std::map<std::pair<int, int>, int> &mergeWordsIdMap);
};
