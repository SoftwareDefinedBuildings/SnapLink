#pragma once

#include "data/Words.h"
#include <memory>
#include <opencv2/core/core.hpp>

class CameraModel;
class Transform;
class Visibility;
class HTTPServer;

class Perspective {
public:
  Perspective(const std::shared_ptr<Words> &words);

  void localize(const std::vector<int> &wordIds,
                const std::vector<cv::KeyPoint> &keyPoints,
                const CameraModel &camera, int &dbId,
                Transform &transform) const;

private:
  static std::map<int, std::vector<cv::KeyPoint>>
  getWords2(const std::vector<int> &wordIds,
            const std::vector<cv::KeyPoint> &keyPoints);
  std::map<int, std::vector<cv::Point3f>>
  getWords3(const std::vector<int> &wordIds, int &dbId) const;
  std::map<int, int>
  countWords(const std::map<int, std::vector<cv::KeyPoint>> &words2,
             const std::map<int, std::vector<cv::Point3f>> &words3) const;
  void getMatchPoints(std::vector<cv::Point3f> &objectPoints,
                      std::vector<cv::Point2f> &imagePoints);
  static Transform
  estimateMotion3DTo2D(const std::map<int, cv::Point3f> &words3A,
                       const std::map<int, cv::KeyPoint> &words2B,
                       const CameraModel &camera, const Transform &guess,
                       std::vector<int> *inliersOut, size_t minInliers);

private:
  std::shared_ptr<Words> _words;
};
