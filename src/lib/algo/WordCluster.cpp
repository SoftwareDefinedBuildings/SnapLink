#include "lib/algo/WordCluster.h"
#include "lib/util/Utility.h"
#include <opencv2/opencv.hpp>

WordCluster::WordCluster(float distRatio) : _distRatio(distRatio) {}

std::map<int, Word>
WordCluster::cluster(const std::vector<int> &roomIds,
                     const std::vector<cv::Point3f> &points3,
                     cv::Mat descriptors) {
  std::map<int, Word> words;

  assert(roomIds.size() == points3.size());
  assert(roomIds.size() == static_cast<unsigned int>(descriptors.rows));

  const unsigned int k = 2; // k nearest neighbors
  cv::Mat wordDescriptors;  // word Id is the row number

  cv::BFMatcher matcher;

  int nextWordId = 0;
  for (int i = 0; i < descriptors.rows; i++) {
    Utility::showProgress(static_cast<float>(i + 1) / descriptors.rows);

    bool newWord = false;

    std::vector<std::vector<cv::DMatch>> matches;
    if (!wordDescriptors.empty()) {
      matcher.knnMatch(descriptors.row(i), wordDescriptors, matches, k);
    }

    assert(matches.size() <= 1);
    if (matches.empty()) {
      newWord = true;
    } else if (matches.at(0).size() < 2) {
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
      wordId = nextWordId;
      nextWordId++;
      words.emplace(wordId, Word(wordId));
    } else {
      wordId = matches.at(0).at(0).queryIdx;
    }
    words.at(wordId).addPoint3(roomIds.at(i), points3.at(i),
                               descriptors.row(i));
    assert(wordId <= wordDescriptors.rows);
    if (wordId == wordDescriptors.rows) {
      wordDescriptors.push_back(words.at(wordId).getMeanDescriptor());
    } else if (wordId < wordDescriptors.rows) {
      wordDescriptors.row(wordId) = words.at(wordId).getMeanDescriptor();
    }
  }
  std::cout << std::endl;

  return words;
}
