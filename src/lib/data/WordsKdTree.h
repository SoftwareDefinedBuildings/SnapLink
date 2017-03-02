#pragma once

#include "lib/data/Words.h"
#include <opencv2/flann.hpp>

class WordsKdTree final : public Words {
public:
  explicit WordsKdTree();

  /**
   * Add words
   */
  void putWords(std::list<std::unique_ptr<Word>> &&words) final;

  /**
   * get all words, indexed by wordId
   */
  const std::map<int, std::shared_ptr<Word>> &getWordsById() const final;

  /**
   * get all words, indexed by dbId, and then wordId
   */
  const std::map<int, std::map<int, std::shared_ptr<Word>>> &
  getWordsByDb() const final;

  /**
   * find the indices of the nearst neighbors of descriptors
   */
  std::vector<int> findNNs(const cv::Mat &descriptors) const final;

private:
  void build();

private:
  int _type;
  int _dim;
  std::map<int, std::shared_ptr<Word>> _wordsById;
  std::map<int, std::map<int, std::shared_ptr<Word>>> _wordsByDb;
  cv::Mat _dataMat;
  std::unique_ptr<cv::flann::Index> _index;
  std::map<int, int> _mapIndexId; // (row num, word id)
};
