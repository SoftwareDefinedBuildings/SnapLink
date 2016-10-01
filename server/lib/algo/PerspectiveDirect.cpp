#include "algo/PerspectiveDirect.h"
#include "data/CameraModel.h"
#include "data/Transform.h"
#include "util/Time.h"
#include "util/Utility.h"
#include <QDebug>
#include <cassert>
#include <pcl/common/transforms.h>

PerspectiveDirect::PerspectiveDirect(const std::shared_ptr<Words> &words)
    : _words(words) {}

void PerspectiveDirect::localize(const std::vector<int> &wordIds,
                                 const std::vector<cv::KeyPoint> &keyPoints,
                                 const CameraModel &camera,
                                 int &dbId,
                                 Transform &transform) const {
  size_t minInliers = 3;

  int inliersCount = 0;

  std::multimap<int, cv::KeyPoint> words2 = createWords(wordIds, keyPoints);

  const std::map<int, std::unique_ptr<Word>> &words = _words->getWords();
  const std::map<int, std::multimap<int, pcl::Point3f>> words3Map; // dbId: words3
  std::map<int, int> dbCounts;
  for (int wordId : wordIds) {
    const auto iter = words.find(wordId);
    assert(iter != words.end());
    const std::unique_ptr<Word> &word = iter->second;
    int dbId = iter->second->getDbId();
    for (const auto &point3 : word->getPoints3()) {
      words3Map[dbId].insert(std::make_pair(wordId, point3));
    }
    auto jter = dbCounts.find(dbId);
    if (jter == dbCounts.end()) {
        dbCounts.insert(std::make_pair(dbId, 1));
    } else {
        jter->second++;
    }
  }
  auto maxCount = std::max_element(dbCounts.begin(), dbCounts.end(), [](const std::pair<int, int>& p1, const std::pair<int, int>& p2) {return p1.second < p2.second; });
  dbId = maxCount->first;
  std::multimap<int, pcl::Point3f> words3 = words3Map[dbId];

  // 3D to 2D (PnP)
  if (words3.size() >= minInliers && words2.size() >= minInliers) {
    std::vector<int> inliers;

    // TODO lots of useful information are thrown away here
    transform = estimateMotion3DTo2D(
        Utility::MultimapToMapUnique(words3),
        Utility::MultimapToMapUnique(words2), camera,
        Transform(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0), // use the old signature's pose as a guess
        &inliers, minInliers);
    inliersCount = (int)inliers.size();
    if (transform.isNull()) {
      std::cout << "Not enough inliers " << inliersCount << "/" << minInliers << std::endl;
    }
  } else {
    std::cout << "Not enough features in images (old=" << words3.size()
              << ", new=" << words.size() << ", min=" << minInliers << ")"
              << std::endl;
  }

  // TODO check RegistrationVis.cpp to see whether rotation check is necessary

  qDebug() << "transform= " << transform.prettyPrint().c_str();
}

std::multimap<int, cv::KeyPoint>
PerspectiveDirect::createWords(const std::vector<int> &wordIds,
                               const std::vector<cv::KeyPoint> &keyPoints) {
  std::multimap<int, cv::KeyPoint> words;
  assert(wordIds.size() == keyPoints.size());
  unsigned int i = 0;
  for (auto iter = wordIds.begin();
       iter != wordIds.end() && i < keyPoints.size(); ++iter, ++i) {
    words.insert(std::pair<int, cv::KeyPoint>(*iter, keyPoints[i]));
  }

  return words;
}

Transform PerspectiveDirect::estimateMotion3DTo2D(
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
                       8.0, 0.99, inliers,
                       cv::SOLVEPNP_ITERATIVE); // cv::SOLVEPNP_EPNP
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

      // TODO: compute variance (like in PCL computeVariance() method of
      // sac_model.h)
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
