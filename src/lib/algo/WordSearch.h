#pragma once

#include "lib/data/Words.h"
#include <memory>

class WordSearch final {
public:
  explicit WordSearch(const std::shared_ptr<Words> &words);

  std::vector<int> search(const cv::Mat &descriptors) const;

private:
  std::shared_ptr<Words> _words;
};
