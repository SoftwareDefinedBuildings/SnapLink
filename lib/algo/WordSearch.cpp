#include "algo/WordSearch.h"
#include <cassert>

WordSearch::WordSearch(const std::shared_ptr<Words> &words) : _words(words) {}

std::vector<int> WordSearch::search(const cv::Mat &descriptors) const {
  assert(descriptors.rows > 0);
  return _words->findNNs(descriptors);
}
