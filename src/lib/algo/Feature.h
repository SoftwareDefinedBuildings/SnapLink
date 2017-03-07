#pragma once

#include <opencv2/xfeatures2d.hpp>

class Feature final {
public:
  explicit Feature(int sampleSize = 0); // 0 means no subsampling

  void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints,
               cv::Mat &descriptors) const;

private:
  void subsample(std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors) const;

private:
  int _sampleSize;
  cv::Ptr<cv::xfeatures2d::SURF> _detector;
};
