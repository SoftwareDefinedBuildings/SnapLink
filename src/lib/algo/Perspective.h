#pragma once

#include "lib/data/Words.h"
#include <memory>
#include <opencv2/core/core.hpp>
#include <set>

class CameraModel;
class Transform;
class Visibility;
class HTTPServer;

class Perspective final {
public:
  explicit Perspective(const std::shared_ptr<Words> &words, int corrSize = 100, double distRatio = 0.7);

  void localize(const std::vector<int> &wordIds,
                const std::vector<cv::KeyPoint> &keyPoints,
                const cv::Mat &descriptors, const CameraModel &camera,
                int dbId, Transform &transform) const;

private:
  static std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
  getWords2(const std::vector<int> &wordIds,
            const std::vector<cv::KeyPoint> &keyPoints,
            const cv::Mat &descriptors);

  /**
   * get 3D point and descriptors, indexed by word Id, from the database
   */
  std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
  getWords3(const std::set<int> &wordIds, int dbId) const;

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
  std::shared_ptr<Words> _words;
  int _corrSize;
  double _distRatio;
};
