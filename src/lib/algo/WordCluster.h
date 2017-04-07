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
  bool _dataChanged;
  std::map<int, Word> _words;
  int _type;
  int _dim;
  std::vector<int> _roomIds;
  std::vector<cv::Point3f> _points3;
  cv::Mat _descriptors;
};
