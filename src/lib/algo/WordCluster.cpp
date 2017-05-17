#include "lib/algo/WordCluster.h"
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
  std::vector<int> wordIds; // word Ids of all points

  int nextWordId = 0;
  for (int i = 0; i < descriptors.rows; i++) {
    bool newWord = false;

    cv::BFMatcher matcher;

    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(descriptors.row(i),
                     descriptors(cv::Range(0, i), cv::Range::all()), matches,
                     k);

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
      wordId = nextWordId;
      nextWordId++;
      words.emplace(wordId, Word(wordId));
    } else {
      wordId = wordIds.at(matches.at(0).at(0).queryIdx);
    }
    words.at(wordId).addPoint3(roomIds.at(i), points3.at(i),
                               descriptors.row(i));
    wordIds.emplace_back(wordId);
  }

  return words;
}
