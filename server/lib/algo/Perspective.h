#pragma once

#include "data/Words.h"
#include <memory>
#include <opencv2/core/core.hpp>

#define MAX_MATCH 50
#define DIST_RATIO 0.7

class CameraModel;
class Transform;
class Visibility;
class HTTPServer;

class Perspective {
public:
  Perspective(const std::shared_ptr<Words> &words);

  void localize(const std::vector<int> &wordIds,
                const std::vector<cv::KeyPoint> &keyPoints,
                const cv::Mat &descriptors, const CameraModel &camera,
                int &dbId, Transform &transform) const;

private:
  static std::map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>>
  getWords2(const std::vector<int> &wordIds,
            const std::vector<cv::KeyPoint> &keyPoints,
            const cv::Mat &descriptors);

  std::map<int, std::pair<std::vector<cv::Point3f>, cv::Mat>>
  getWords3(const std::vector<int> &wordIds, int &dbId) const;

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

  static bool
  findMatchPoint3(const cv::Mat &descriptor,
                  const std::pair<std::vector<cv::Point3f>, cv::Mat> &words3,
                  cv::Point3f &point3);

  static Transform solvePnP(const std::vector<cv::Point2f> &imagePoints,
                            const std::vector<cv::Point3f> &objectPoints,
                            const CameraModel &camera);

private:
  std::shared_ptr<Words> _words;
};
