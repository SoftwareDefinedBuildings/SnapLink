#pragma once

#include "data/Words.h"
#include <memory>

class WordSearch final {
public:
  WordSearch(const std::shared_ptr<Words> &words);

  std::vector<int> search(const cv::Mat &descriptors) const;

private:
  std::shared_ptr<Words> _words;
};
