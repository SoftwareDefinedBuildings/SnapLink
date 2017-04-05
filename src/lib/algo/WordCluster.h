#pragma once

#include "lib/data/Word.h"
#include <memory>
#include <opencv2/flann.hpp>

class WordCluster final {
public:
  explicit WordCluster(int distRatio);

  void addDescriptors(cv::Mat descriptors);
  const std::map<int, Word> &getWords();

private:
  void createWords();
  void buildIndex();

private:
  double _distRatio;
  bool _hasNewData;
  std::map<int, Word> _words;
  int _type;
  int _dim;
  cv::Mat _descriptors;
  std::unique_ptr<cv::flann::Index> _index;
};
