#pragma once

#include "data/Words.h"
#include <opencv2/flann.hpp>

class WordsKdTree : public Words {
public:
  WordsKdTree();

  /**
   * Add words
   */
  void putWords(std::list<std::unique_ptr<Word>> &&words);

  /**
   * get all words
   */
  const std::map<int, std::unique_ptr<Word>> &getWords() const;

  /**
   * find the indices of the nearst neighbors of descriptors
   */
  std::vector<int> findNNs(const cv::Mat &descriptors) const;

private:
  void build();

private:
  int _type;
  int _dim;
  std::map<int, std::unique_ptr<Word>> _words;
  cv::Mat _dataMat;
  std::unique_ptr<cv::flann::Index> _index;
  std::map<int, int> _mapIndexId; // (row num, word id)
};
