#include "lib/algo/WordCluster.h"

WordCluster::WordCluster(int distRatio)
    : _distRatio(distRatio), _hasNewData(false), _type(-1), _dim(-1) {}

WordCluster::addDescriptors(cv::Mat descriptors) {
  if (_type < 0 || _dim < 0) {
    _type = descriptor.type();
    _dim = descriptor.cols;
  }
  assert(descriptor.type() == _type);
  assert(descriptor.cols == _dim);

  // will incur data copy
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
  cv::Mat id2dist;
  std::vector<std::vector<cv::DMatch>> matches;

  _index = std::make_unique<cv::flann::Index>(_descriptors,
                                              cv::flann::KDTreeIndexParams());

  _index->knnSearch(descriptors, indices, id2dist, k);
  assert(id2dist.cols == 2);

  for (int id = 1; id < descriptors.rows + 1; id++) {
    std::multimap<float, int> dist2id;
    for (int j = 0; j < id2dist.cols; ++j) {
      float d = id2dist.at<float>(id, j);
      if (d >= 0.0f) {
        dist2id.insert(std::pair<float, int>(d, id));
      } else {
        break;
      }
    }

    if (_incrementalDictionary) {
      bool badDist = false;
      if (dist2id.size() == 0) {
        badDist = true;
      }
      if (!badDist) {
        if (dist2id.size() >= 2) {
          // Apply NNDR
          if (dist2id.begin()->first >
              _nndrRatio * (++dist2id.begin())->first) {
            badDist = true; // Rejected
          }
        } else {
          badDist = true; // Rejected
        }
      }

      if (badDist) {
        // use original descriptor
        VisualWord *vw =
            new VisualWord(getNextId(), descriptorsIn.row(i), signatureId);
        _visualWords.insert(_visualWords.end(),
                            std::pair<int, VisualWord *>(vw->id(), vw));
        _notIndexedWords.insert(_notIndexedWords.end(), vw->id());
        newWords.push_back(descriptors.row(i));
        newWordsId.push_back(vw->id());
        wordIds.push_back(vw->id());
        UASSERT(vw->id() > 0);
      } else {
        if (_notIndexedWords.find(dist2id.begin()->second) !=
            _notIndexedWords.end()) {
          ++dupWordsCountFromLast;
        } else {
          ++dupWordsCountFromDict;
        }

        this->addWordRef(dist2id.begin()->second, signatureId);
        wordIds.push_back(dist2id.begin()->second);
      }
    } else if (dist2id.size()) {
      // If the dictionary is not incremental, just take the nearest word
      ++dupWordsCountFromDict;
      this->addWordRef(dist2id.begin()->second, signatureId);
      wordIds.push_back(dist2id.begin()->second);
      UASSERT(dist2id.begin()->second > 0);
    }
  }

  _totalActiveReferences += _notIndexedWords.size();
  return wordIds;
}
