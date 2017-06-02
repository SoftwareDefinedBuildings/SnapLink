#pragma once

#include "lib/data/Word.h"

#define DIST_RATIO 0.7

class WordCluster final {
public:
  explicit WordCluster(float distRatio = DIST_RATIO);

  std::map<int, Word> cluster(const std::vector<int> &roomIds,
                              const std::vector<cv::Point3f> &points3,
                              cv::Mat descriptors);

private:
  float _distRatio;
};
