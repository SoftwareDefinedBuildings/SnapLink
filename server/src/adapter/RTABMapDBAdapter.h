#pragma once

#include "data/Labels.h"
#include "data/Signatures.h"
#include "data/Words.h"
#include <list>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <rtabmap/core/Transform.h>
#include <set>
#include <typeinfo>

class Words;

class RTABMapDBAdapter {
public:
  /**
   * read data from database files, NULL pointers will be ignored
   */
  static bool readData(const std::vector<std::string> &dbPaths, Words &words,
                       Signatures &signatures, Labels &labels);

private:
  static std::list<std::unique_ptr<Signature>>
  readSignatures(const std::string &dbPath, int dbId);
  static std::list<std::unique_ptr<Word>>
  readWords(const std::string &dbPath,
            const std::list<std::unique_ptr<Signature>> &signatures);
  static std::list<std::unique_ptr<Label>>
  readLabels(const std::string &dbPath, int dbId,
             const std::list<std::unique_ptr<Signature>> &signatures);

  static std::map<int, rtabmap::Transform>
  getOptimizedPoseMap(const std::string &dbPath);

  static bool
  getPoint3World(const std::list<std::unique_ptr<Signature>> &signatures,
                 int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);

  static std::map<std::pair<int, int>, int> getMergeWordsIdMap(
      const std::map<int, std::list<std::unique_ptr<Word>>> &wordsMap);
  static std::map<std::pair<int, int>, int> getMergeSignaturesIdMap(
      const std::list<std::unique_ptr<Signature>> &signatures);
  static std::list<std::unique_ptr<Word>>
  mergeWords(std::map<int, std::list<std::unique_ptr<Word>>> &&wordsMap,
             const std::map<std::pair<int, int>, int> &mergeWordsIdMap,
             const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap);
  static std::list<std::unique_ptr<Signature>> mergeSignatures(
      std::list<std::unique_ptr<Signature>> &&signatures,
      const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap,
      const std::map<std::pair<int, int>, int> &mergeWordsIdMap);
};
