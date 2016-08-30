#pragma once

#include <opencv2/xfeatures2d.hpp>

class WordSearch;

class Feature {
public:
  Feature();

  void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints,
               cv::Mat &descriptors) const;

private:
  cv::Ptr<cv::xfeatures2d::SURF> _detector;
};
