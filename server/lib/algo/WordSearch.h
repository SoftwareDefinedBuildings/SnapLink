#pragma once

#include "data/Words.h"
#include <memory>

class WordSearch {
public:
  WordSearch(std::unique_ptr<Words> &&words);

  std::vector<int> search(const cv::Mat &descriptors) const;

private:
  std::unique_ptr<Words> _words;
};
