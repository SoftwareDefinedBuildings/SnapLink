#include "data/WordsKdTree.h"
#include <cassert>

WordsKdTree::WordsKdTree() : _type(-1), _dim(-1) {}

void WordsKdTree::putWords(std::list<std::unique_ptr<Word>> &&words) {
  for (auto &word : words) {
    assert(word != nullptr);
    int wordId = word->getId();
    int dbId = word->getDbId();
    std::shared_ptr<Word> tempPtr(std::move(word));
    _wordsById[wordId] = tempPtr;
    _wordsByDb[dbId][wordId] = tempPtr;
  }
  build();
}

const std::map<int, std::shared_ptr<Word>> &WordsKdTree::getWordsById() const {
  return _wordsById;
}

const std::map<int, std::map<int, std::shared_ptr<Word>>> &
WordsKdTree::getWordsByDb() const {
  return _wordsByDb;
}

std::vector<int> WordsKdTree::findNNs(const cv::Mat &descriptors) const {
  int k = 1;
  std::vector<int> resultIds(descriptors.rows, 0);

  if (_wordsById.size() && descriptors.rows) {
    // verify we have the same features
    assert(_type == descriptors.type());
    assert(_dim == descriptors.cols);

    cv::Mat indices;
    cv::Mat dists;
    if (_index != nullptr) {
      // Find nearest neighbors
      indices.create(descriptors.rows, k, CV_32S);
      dists.create(descriptors.rows, k, CV_32F);

      _index->knnSearch(descriptors, indices, dists, k,
                        cv::flann::SearchParams(128));
    }

    assert(dists.rows == descriptors.rows && dists.cols == k);
    for (int i = 0; i < descriptors.rows; ++i) {
      float d = dists.at<float>(i, 0);
      int id = _mapIndexId.at(indices.at<int>(i, 0));
      assert(d >= 0.0f && id > 0);
      resultIds[i] = id;
    }
  }
  return resultIds;
}

void WordsKdTree::build() {
  _mapIndexId.clear();
  _dataMat = cv::Mat();

  if (_wordsById.size()) {
    // use the first word to define the type and dim
    if (_type < 0 || _dim < 0) {
      const cv::Mat &descriptor = _wordsById.begin()->second->getDescriptor();
      _type = descriptor.type();
      _dim = descriptor.cols;
    }

    // Create the data matrix
    _dataMat = cv::Mat(_wordsById.size(), _dim, _type);
    int i = 0;
    for (const auto &word : _wordsById) {
      const cv::Mat &descriptor = word.second->getDescriptor();

      assert(descriptor.type() == _type);
      assert(descriptor.cols == _dim);

      descriptor.copyTo(_dataMat.row(i));
      _mapIndexId.insert(_mapIndexId.end(), std::make_pair(i, word.first));
      i++;
    }

    _index.reset(
        new cv::flann::Index(_dataMat, cv::flann::KDTreeIndexParams(4)));
  }
}
