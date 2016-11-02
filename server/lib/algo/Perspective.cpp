#include "algo/Perspective.h"
#include "data/CameraModel.h"
#include "data/Transform.h"
#include "util/Time.h"
#include "util/Utility.h"
#include <QDebug>
#include <cassert>
#include <pcl/common/transforms.h>

Perspective::Perspective(const std::shared_ptr<Words> &words) : _words(words) {}

void Perspective::localize(const std::vector<int> &wordIds,
                           const std::vector<cv::KeyPoint> &keyPoints,
                           const CameraModel &camera, int &dbId,
                           Transform &transform) const {
  std::map<int, std::vector<cv::KeyPoint>> words2 =
      getWords2(wordIds, keyPoints);
  std::map<int, std::vector<cv::Point3f>> words3 = getWords3(wordIds, dbId);
  std::cout << "words2.size() = " << words2.size()
            << ", words3.size() = " << words3.size() << std::endl;

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

  // TODO check RegistrationVis.cpp to see whether rotation check is necessary

  qDebug() << "transform= " << transform.prettyPrint().c_str();
}

std::map<int, std::vector<cv::KeyPoint>>
Perspective::getWords2(const std::vector<int> &wordIds,
                       const std::vector<cv::KeyPoint> &keyPoints) {
  std::map<int, std::vector<cv::KeyPoint>> words;
  assert(wordIds.size() == keyPoints.size());
  unsigned int i = 0;
  for (const auto wordId : wordIds) {
    // an empty vector is ceated if wordId is not in words3Map[dbId]
    words[wordId].emplace_back(keyPoints[i]);
    i++;
  }

  return words;
}

std::map<int, std::vector<cv::Point3f>>
Perspective::getWords3(const std::vector<int> &wordIds, int &dbId) const {
  const std::map<int, std::shared_ptr<Word>> &wordsById =
      _words->getWordsById();
  std::map<int, std::map<int, std::vector<cv::Point3f>>>
      words3Map;               // dbId: wordId: point3
  std::map<int, int> dbCounts; // dbId: count
  for (int wordId : wordIds) {
    const auto iter = wordsById.find(wordId);
    assert(iter != wordsById.end());
    const std::shared_ptr<Word> &word = iter->second;
    for (auto &points3 : word->getPoints3Map()) {
      int dbId = points3.first;
      for (const auto &point3 : points3.second) {
        // an empty vector is ceated if wordId is not in words3Map[dbId]
        words3Map[dbId][wordId].emplace_back(point3);
      }
      auto jter = dbCounts.find(dbId);
      if (jter == dbCounts.end()) {
        auto ret = dbCounts.emplace(dbId, 0);
        jter = ret.first;
      }
      jter->second++; // TODO: +1 or +points3.size() ?
    }
  }
  std::map<int, double> dbCountsNorm;
  for (const auto &count : dbCounts) {
    dbCountsNorm[count.first] =
        (float)count.second / _words->getWordsByDb().at(count.first).size();
    std::cout << "dbId = " << count.first
              << ", norm count = " << dbCountsNorm[count.first] << std::endl;
  }
  auto maxCount = std::max_element(dbCountsNorm.begin(), dbCountsNorm.end(),
                                   [](const std::pair<double, double> &p1,
                                      const std::pair<double, double> &p2) {
                                     return p1.second < p2.second;
                                   });
  dbId = maxCount->first;
  std::cout << "max dbId = " << dbId << std::endl;

  return words3Map[dbId];
}

std::map<int, int> Perspective::countWords(
    const std::map<int, std::vector<cv::KeyPoint>> &words2,
    const std::map<int, std::vector<cv::Point3f>> &words3) const {
  std::map<int, int> counts;

  for (const auto word : words2) {
    counts.emplace(word.first, word.second.size());
  }
  for (const auto word : words3) {
    counts.at(word.first) += word.second.size();
  }

  return counts;
}

void Perspective::getMatchPoints(
    const std::map<int, std::vector<cv::KeyPoint>> &words2,
    const std::map<int, std::vector<cv::Point3f>> &words3,
    std::vector<cv::Point2f> &imagePoints,
    std::vector<cv::Point3f> &objectPoints) const {
  std::map<int, int> wordCounts =
      countWords(words2, words3); // wrod id -> count of both words2 and words3
  std::vector<std::pair<int, int>> inverseCounts;
  for (const auto &count : wordCounts) {
    inverseCounts.emplace_back(count.second, count.first);
  }
  std::sort(inverseCounts.begin(), inverseCounts.end());

  for (const auto &count : inverseCounts) {
    int wordId = count.second;
    if (count.first == 2) {
      imagePoints.emplace_back(words2.at(wordId).at(0).pt);
      objectPoints.emplace_back(words3.at(wordId).at(0));
    }
    // TODO: VPS
  }
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

    // compute variance, which is the rms of reprojection errors
    // TODO
  }

  return transform;
}
