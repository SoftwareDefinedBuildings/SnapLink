#pragma once

#include "lib/data/Words.h"
#include <memory>

class DbSearch final {
public:
  explicit DbSearch(const std::shared_ptr<Words> &words);

  /**
   * return the id of the most similar database.
   */
  int search(const std::vector<int> &wordIds) const;

private:
  std::shared_ptr<Words> _words;
};
