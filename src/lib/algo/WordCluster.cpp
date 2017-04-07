#include "lib/algo/WordCluster.h"

WordCluster::WordCluster(int distRatio)
    : _distRatio(distRatio), _hasNewData(false), _type(-1), _dim(-1) {}

void WordCluster::addDescriptors(std::vector<int> &&roomIds,
                                 std::vector<cv::Point3f> &&points3,
                                 cv::Mat descriptors) {
  if (_type < 0 || _dim < 0) {
    _type = descriptor.type();
    _dim = descriptor.cols;
  }
  assert(descriptor.type() == _type);
  assert(descriptor.cols == _dim);

  assert(roomIds.size() == points3.size());
  assert(roomIds.size() == descriptor.rows);

  // will incur data copy
  std::move(roomIds.begin(), roomIds.end(), std::back_inserter(_roomIds));
  std::move(points3.begin(), points3.end(), std::back_inserter(_points3));
  cv::vconcat(_descriptors, descriptors, _descriptors);

  _hasNewData = true;
}

const std::map<int, Word> &WordCluster::getWords() {
  if (_hasNewData) {
    createWords();
    _hasNewData = false;
  }
  return _words;
}

void WordCluster::createWords() {
  assert(!descriptors.empty());

  int dupWordsCountFromDict = 0;
  int dupWordsCountFromLast = 0;

  const unsigned int k = 2; // k nearest neighbors

  cv::Mat newWords;
  std::vector<int> newWordsId;

  cv::Mat indices;
  cv::Mat dists;
  std::vector<std::vector<cv::DMatch>> matches;

  assert(!dists.empty());
  assert(dists.cols == 2);

  int nextId = 1;
  for (int i = 0; i < descriptors.rows; i++) {
    bool newWord = false;

    cv::BFMatcher::BFMatcher matcher;

    std::vector<std::vector<cv::DMatch>> matches;
    macher.knnMatch(descriptors.row(i),
                    descriptors(cv::Range(0, i), cv::Range::all()), matches, k);

    std::multimap<float, int> dist2id;
    for (int j = 0; j < dists.cols; ++j) {
      float d = dists.at<float>(id, j);
      if (d >= 0.0f) {
        dist2id.insert(std::pair<float, int>(d, id));
      } else {
        break;
      }
    }

    // Apply NNDR
    float d1 = dists.at<float>(id, 0);
    float d2 = dists.at<float>(id, 1);
    if (d1 > _distRatio * d2) {
      newWord = true;
    }

    int wordId;
    if (newWord) {
      wordId = nextId;
      nextId++;
      _words.emplace(wordId, Word(wordId));
    } else {
    }
    _words.at(wordId).addPoint3(_roomIds.at(i), _points3.at(i),
                                descriptors.row(i));
  }

  _totalActiveReferences += _notIndexedWords.size();
  return wordIds;
}
