#pragma once

#include "Word.h"
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

class Words {
public:
  virtual ~Words() = default;

  /**
   * Add words, ownership transfer
   */
  virtual void putWords(std::list<std::unique_ptr<Word>> &&words) = 0;

  /**
   * find the indices of the nearst neighbors of descriptors
   */
  virtual std::vector<int> findNNs(const cv::Mat &descriptors) const = 0;
};
