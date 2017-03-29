#include "lib/algo/Perspective.h"
#include "lib/data/CameraModel.h"
#include "lib/data/Transform.h"
#include "lib/util/Utility.h"
#include <cassert>
#include <pcl/common/transforms.h>

Perspective::Perspective(const std::map<int, Room> &rooms,
                         const std::map<int, Word> &words, int corrLimit,
                         double distRatio)
    : _rooms(rooms), _words(words), _corrLimit(corrLimit),
      _distRatio(distRatio) {}

void Perspective::localize(const std::vector<int> &wordIds,
                           const std::vector<cv::KeyPoint> &keyPoints,
                           const cv::Mat &descriptors,
                           const CameraModel &camera, int roomId,
                           Transform &transform) const {
  if (wordIds.size() == 0) {
    return;
  }

  std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> words2 =
      getWords2(wordIds, keyPoints, descriptors);
  std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> words3 =
      getWords3(std::set<int>(wordIds.begin(), wordIds.end()), roomId);

  std::vector<cv::Point2f> imagePoints;
  std::vector<cv::Point3f> objectPoints;
  getMatchPoints(words2, words3, imagePoints, objectPoints);
  std::cout << "imagePoints.size() = " << imagePoints.size()
            << ", objectPoints.size() = " << objectPoints.size() << std::endl;

  // 3D to 2D (PnP)
  transform = solvePnP(imagePoints, objectPoints, camera);
  if (transform.isNull()) {
    std::cout << "Localization failed" << std::endl;
  }

  std::cout << "transform :" << std::endl << transform << std::endl;
}

std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
Perspective::getWords2(const std::vector<int> &wordIds,
                       const std::vector<cv::KeyPoint> &keyPoints,
                       const cv::Mat &descriptors) {
  std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> words2;
  assert(wordIds.size() == keyPoints.size());
  unsigned int i = 0;
  for (const auto wordId : wordIds) {
    // an empty vector is ceated if wordId is not in words2
    words2[wordId].first.emplace_back(keyPoints[i]);
    words2[wordId].second.push_back(descriptors.row(i));
    i++;
  }

  return words2;
}

std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
Perspective::getWords3(const std::set<int> &wordIds, int roomId) const {
  std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
      words3; // wordId: point3

  const auto &roomWords = _rooms.at(roomId).getWordIds();

  for (int wordId : wordIds) {
    auto iter = roomWords.find(wordId);
    if (iter == roomWords.end()) {
      continue;
    }

    const auto jter = _words.find(wordId);
    assert(jter != _words.end());
    const Word &word = jter->second;
    const auto &points3 = word.getPoints3Map().at(roomId);
    cv::Mat desc = word.getDescriptorsByDb().at(roomId);

    unsigned int i = 0;
    for (const auto &point3 : points3) {
      // an empty vector is ceated if wordId is not in words3
      words3[wordId].first.emplace_back(point3);
      words3[wordId].second.push_back(desc.row(i));
      i++;
    }
  }

  return words3;
}

std::map<int, int> Perspective::countWords(
    const std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> &words2,
    const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> &words3)
    const {
  std::map<int, int> counts;

  for (const auto word : words2) {
    counts.emplace(word.first, word.second.first.size());
  }
  for (const auto word : words3) {
    counts.at(word.first) += word.second.first.size();
  }

  return counts;
}

void Perspective::getMatchPoints(
    const std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> &words2,
    const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> &words3,
    std::vector<cv::Point2f> &imagePoints,
    std::vector<cv::Point3f> &objectPoints) const {
  std::map<int, int> wordCounts =
      countWords(words2, words3); // word id -> count of both words2 and words3
  std::vector<std::pair<int, int>> inverseCounts;
  for (const auto &count : wordCounts) {
    inverseCounts.emplace_back(count.second, count.first);
  }
  std::sort(inverseCounts.begin(), inverseCounts.end());

  int matchCount = 0;
  // iterate from word with less points
  for (const auto &count : inverseCounts) {
    int wordId = count.second;
    unsigned int i = 0;

    assert(words2.find(wordId) != words2.end());
    if (words3.find(wordId) == words3.end()) {
      continue;
    }

    for (const auto &point2 : words2.at(wordId).first) {
      cv::Point3f point3;
      cv::Mat desc = words2.at(wordId).second.row(i);
      if (findMatchPoint3(desc, wordId, words3, point3)) {
        imagePoints.emplace_back(point2.pt);
        objectPoints.emplace_back(point3);
        matchCount++;
        if (_corrLimit > 0 && matchCount >= _corrLimit) {
          return;
        }
      }
      i++;
    }
  }
}

bool Perspective::findMatchPoint3(
    const cv::Mat &descriptor, int wordId,
    const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> &words3,
    cv::Point3f &point3) const {
  assert(descriptor.rows == 1);

  if (words3.find(wordId) == words3.end()) {
    return false;
  }

  if (words3.at(wordId).first.size() == 1) {
    point3 = words3.at(wordId).first.at(0);
    return true;
  }

  std::vector<std::pair<double, int>> dists;
  for (int i = 0; i < words3.at(wordId).second.rows; i++) {
    double dist =
        cv::norm(descriptor, words3.at(wordId).second.row(i), cv::NORM_L2);
    dists.emplace_back(dist, i);
  }
  std::partial_sort(dists.begin(), dists.begin() + 2, dists.end());
  if (dists.at(0).first / dists.at(1).first <= _distRatio) {
    point3 = words3.at(wordId).first.at(dists.at(0).second);
    return true;
  }

  return false;
}

Transform Perspective::solvePnP(const std::vector<cv::Point2f> &imagePoints,
                                const std::vector<cv::Point3f> &objectPoints,
                                const CameraModel &camera) {
  Transform transform;

  assert(imagePoints.size() == objectPoints.size());
  if (imagePoints.size() == 0 || objectPoints.size() == 0) {
    return transform;
  }

  // PnPRansac
  cv::Mat K = camera.K();
  cv::Mat D = camera.D();
  cv::Mat rvec(1, 3, CV_64FC1);
  cv::Mat tvec;

  bool useExtrinsicGuess = false;
  int iterationsCount = 100;
  float reprojectionError = 8.0;
  double confidence = 0.99;
  std::vector<int> inliers;

  bool success =
      cv::solvePnPRansac(objectPoints, imagePoints, K, D, rvec, tvec,
                         useExtrinsicGuess, iterationsCount, reprojectionError,
                         confidence, inliers, cv::SOLVEPNP_EPNP);
  // TODO check RTABMap refine model code

  if (success) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Transform pnp(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                  tvec.at<double>(0), //
                  R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                  tvec.at<double>(1), //
                  R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                  tvec.at<double>(2));

    transform = std::move(pnp);

    // TODO: compute variance, which is the rms of reprojection errors
  }

  return transform;
}
