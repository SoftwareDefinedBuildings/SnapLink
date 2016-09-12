#include "algo/WordSearch.h"
#include <cassert>

WordSearch::WordSearch(std::unique_ptr<Words> &&words)
    : _words(std::move(words)) {}

std::vector<int> WordSearch::search(const cv::Mat &descriptors) const {
  assert(descriptors.rows > 0);
  return _words->findNNs(descriptors);
}
