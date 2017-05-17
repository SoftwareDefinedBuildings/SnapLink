#include "lib/algo/WordCluster.h"

WordCluster::WordCluster(int distRatio)
    : _distRatio(distRatio), _type(-1), _dim(-1), _nextIndex(0) {}

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
}

const std::map<int, Word> &WordCluster::getWords() {
  if (_descriptors.rows > _nextIndex) {
    createWords();
  }
  return _words;
}

void WordCluster::createWords() {
  assert(!descriptors.empty());

  const unsigned int k = 2; // k nearest neighbors

  std::vector<std::vector<cv::DMatch>> matches;

  int nextId = 0;
  for (int i = _nextIndex; i < descriptors.rows; i++) {
    bool newWord = false;

    cv::BFMatcher::BFMatcher matcher;

    std::vector<std::vector<cv::DMatch>> matches;
    macher.knnMatch(descriptors.row(i),
                    descriptors(cv::Range(0, i), cv::Range::all()), matches, k);

    assert(matches.size() == 1);
    if (matches.at(0).size() < 2) {
      newWord = true;
    } else {
      // Apply NNDR
      float d1 = matches.at(0).at(0).distance;
      float d2 = matches.at(0).at(1).distance;
      assert(d1 <= d2);
      if (d1 > _distRatio * d2) {
        newWord = true;
      }
    }

    int wordId;
    if (newWord) {
      wordId = nextId;
      nextId++;
      _words.emplace(wordId, Word(wordId));
    } else {
      wordId = indexWordIdMap.at(matches.at(0).at(0).queryIdx);
    }
    _words.at(wordId).addPoint3(_roomIds.at(i), _points3.at(i),
                                descriptors.row(i));
  }
}
