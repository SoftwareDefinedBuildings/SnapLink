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
  size_t minInliers = 3;

  int inliersCount = 0;

  std::map<int, std::vector<cv::KeyPoint>> words2 = getWords2(wordIds, keyPoints);
  std::map<int, std::vector<cv::Point3f>> words3 = getWords3(wordIds, dbId);
  std::cout << "words3.size() = " << words3.size()
            << ", words2.size() = " << words2.size() << std::endl;

  std::map<int, int> wordCounts = countWords(words2, words3); // wrod id -> count of both words2 and words3
  std::vector<std::pair<int, int>> inverseCounts;
  for (const auto count : wordCounts) {
    inverseCounts.emplace_back(count);
  }
  std::sort(inverseCounts.begin(), inverseCounts.end());

  // 3D to 2D (PnP)
  if (words3.size() >= minInliers && words2.size() >= minInliers) {
    std::vector<int> inliers;

    // TODO lots of useful information are thrown away here
    transform = estimateMotion3DTo2D(
        words3, words2, camera, Transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0),
        &inliers, minInliers);
    inliersCount = (int)inliers.size();
    if (transform.isNull()) {
      std::cout << "Not enough inliers " << inliersCount << "/" << minInliers
                << std::endl;
    }
  } else {
    std::cout << "Not enough features in images (old=" << words3.size()
              << ", new=" << words2.size() << ", min=" << minInliers << ")"
              << std::endl;
  }

  // TODO check RegistrationVis.cpp to see whether rotation check is necessary

  qDebug() << "transform= " << transform.prettyPrint().c_str();
}

std::map<int, int> Perspective::countWords(const std::map<int, std::vector<cv::KeyPoint>> &words2, const std::map<int, std::vector<cv::Point3f>> &words3) const {
  std::map<int, int> counts;

  for (const auto word : words2) {
    counts.emplace(word.first, word.second.size());
  }
  for (const auto word : words3) {
    counts.at(word.first) += word.second.size();
  }

  return counts;
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
        auto ret = dbCounts.insert(std::make_pair(dbId, 1));
        jter = ret.first;
      }
      jter->second++;
    }
  }
  std::map<int, double> dbCountsNorm;
  for (auto count : dbCounts) {
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

std::map<int, std::vector<cv::KeyPoint>>
Perspective::getWords2(const std::vector<int> &wordIds,
                       const std::vector<cv::KeyPoint> &keyPoints) {
  std::map<int, cv::KeyPoint> words;
  assert(wordIds.size() == keyPoints.size());
  unsigned int i = 0;
  for (const auto wordId : wordIds) {
    // an empty vector is ceated if wordId is not in words3Map[dbId]
    words[wordId].emplace(keyPoints[i]);
    i++;
  }

  return words;
}

Transform Perspective::estimateMotion3DTo2D(
    const std::map<int, cv::Point3f> &words3A,
    const std::map<int, cv::KeyPoint> &words2B, const CameraModel &camera,
    const Transform &guess, std::vector<int> *inliersOut, size_t minInliers) {
  assert(!guess.isNull());
  Transform transform;
  std::vector<int> matches, inliers;

  // find correspondences
  std::vector<int> ids = Utility::Keys(words2B);
  std::vector<cv::Point3f> objectPoints(ids.size());
  std::vector<cv::Point2f> imagePoints(ids.size());
  int oi = 0;
  matches.resize(ids.size());
  for (unsigned int i = 0; i < ids.size(); ++i) {
    std::map<int, cv::Point3f>::const_iterator iter = words3A.find(ids[i]);
    if (iter != words3A.end()) {
      const cv::Point3f &pt = iter->second;
      objectPoints[oi].x = pt.x;
      objectPoints[oi].y = pt.y;
      objectPoints[oi].z = pt.z;
      imagePoints[oi] = words2B.find(ids[i])->second.pt;
      matches[oi++] = ids[i];
    }
  }

  objectPoints.resize(oi);
  imagePoints.resize(oi);
  matches.resize(oi);

  qDebug() << "words3A=" << words3A.size() << " words2B= " << words2B.size()
           << " matches=" << matches.size();

  if (matches.size() >= minInliers) {
    // PnPRansac
    cv::Mat K = camera.K();
    cv::Mat D = camera.D();
    cv::Mat R =
        (cv::Mat_<double>(3, 3) << (double)guess.r11(), (double)guess.r12(),
         (double)guess.r13(),                                           //
         (double)guess.r21(), (double)guess.r22(), (double)guess.r23(), //
         (double)guess.r31(), (double)guess.r32(), (double)guess.r33());

    cv::Mat rvec(1, 3, CV_64FC1);
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(1, 3) << (double)guess.x(),
                    (double)guess.y(), (double)guess.z());

    cv::solvePnPRansac(objectPoints, imagePoints, K, D, rvec, tvec, true, 100,
                       8.0, 0.99, inliers, cv::SOLVEPNP_EPNP);
    // TODO check RTABMAp refine model code

    if (inliers.size() >= minInliers) {
      cv::Rodrigues(rvec, R);
      Transform pnp(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    tvec.at<double>(0), //
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    tvec.at<double>(1), //
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                    tvec.at<double>(2));

      transform = std::move(pnp);

      // compute variance, which is the rms of reprojection errors
      std::vector<cv::Point2f> imagePointsReproj;
      cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(),
                        imagePointsReproj);
      float err = 0.0f;
      for (unsigned int i = 0; i < inliers.size(); ++i) {
        err += std::abs(std::complex<double>(
            imagePoints.at(inliers[i]).x - imagePointsReproj.at(inliers[i]).x,
            imagePoints.at(inliers[i]).y - imagePointsReproj.at(inliers[i]).y));
      }
      double varianceOut = std::sqrt(err / float(inliers.size()));
      std::cout << "variance = " << varianceOut << std::endl;
    }
  }

  if (inliersOut) {
    inliersOut->resize(inliers.size());
    for (unsigned int i = 0; i < inliers.size(); ++i) {
      inliersOut->at(i) = matches[inliers[i]];
    }
  }

  return transform;
}
