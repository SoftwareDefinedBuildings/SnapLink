#pragma once

#include "data/Signatures.h"
#include <memory>
#include <opencv2/core/core.hpp>

class CameraModel;
class Transform;
class Visibility;
class HTTPServer;

class PerspectiveDirect {
public:
  PerspectiveDirect(const std::shared_ptr<Signatures> &signatures);

  void localize(const std::vector<int> &wordIds,
                const std::vector<cv::KeyPoint> &keyPoints,
                const CameraModel &camera, 
                Transform &transform) const;

private:
  static std::multimap<int, cv::KeyPoint>
  createWords(const std::vector<int> &wordIds,
              const std::vector<cv::KeyPoint> &keyPoints);
  static Transform
  estimateMotion3DTo2D(const std::map<int, cv::Point3f> &words3A,
                       const std::map<int, cv::KeyPoint> &words2B,
                       const CameraModel &camera, const Transform &guess,
                       std::vector<int> *inliersOut, size_t minInliers);

private:
  std::shared_ptr<Signatures> _signatures;
};
