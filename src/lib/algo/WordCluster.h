#pragma once

#include "lib/data/Word.h"
#include <memory>

class WordCluster final {
public:
  explicit WordCluster(int distRatio);

  void addDescriptors(std::vector<int> &&roomId,
                      std::vector<cv::Point3f> &&points3, cv::Mat descriptors);
  const std::map<int, Word> &getWords();

private:
  void createWords();
  void buildIndex();

private:
  double _distRatio;
  int _type;
  int _dim;
  int _nextIndex; // index of next data to process
  std::map<int, Word> _words;
  std::vector<int> _roomIds;
  std::vector<cv::Point3f> _points3;
  std::map<int, int> indexWordIdMap;
  cv::Mat _descriptors;
};
