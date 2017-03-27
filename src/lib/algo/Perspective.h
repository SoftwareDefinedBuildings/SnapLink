#pragma once

#include "lib/data/Room.h"
#include "lib/data/Word.h"
#include <memory>
#include <opencv2/core/core.hpp>
#include <set>

#define CORR_LIMIT 0
#define DIST_RATIO 0.7

class CameraModel;
class Transform;

class Perspective final {
public:
  explicit Perspective(const std::map<int, Room> &rooms,
                       const std::map<int, Word> &words,
                       int corrLimit = CORR_LIMIT,
                       double distRatio = DIST_RATIO);

  void localize(const std::vector<int> &wordIds,
                const std::vector<cv::KeyPoint> &keyPoints,
                const cv::Mat &descriptors, const CameraModel &camera,
                int roomId, Transform &transform) const;

private:
  static std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
  getWords2(const std::vector<int> &wordIds,
            const std::vector<cv::KeyPoint> &keyPoints,
            const cv::Mat &descriptors);

  /**
   * get 3D point and descriptors, indexed by word Id, from the database
   */
  std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
  getWords3(const std::set<int> &wordIds, int roomId) const;

  std::map<int, int>
  countWords(const std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
                 &words2,
             const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
                 &words3) const;

  void getMatchPoints(
      const std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
          &words2,
      const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> &words3,
      std::vector<cv::Point2f> &imagePoints,
      std::vector<cv::Point3f> &objectPoints) const;

  bool findMatchPoint3(
      const cv::Mat &descriptor, int wordId,
      const std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>> &words3,
      cv::Point3f &point3) const;

  static Transform solvePnP(const std::vector<cv::Point2f> &imagePoints,
                            const std::vector<cv::Point3f> &objectPoints,
                            const CameraModel &camera);

private:
  const std::map<int, Room> &_rooms;
  const std::map<int, Word> &_words;
  int _corrLimit;
  double _distRatio;
};
