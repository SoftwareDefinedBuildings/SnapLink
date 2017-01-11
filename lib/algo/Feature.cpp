#include "algo/Feature.h"

Feature::Feature(int sampleSize) : _sampleSize(sampleSize) {
  int minHessian = 400;
  _detector = cv::xfeatures2d::SURF::create(minHessian);
}

void Feature::extract(const cv::Mat &image,
                      std::vector<cv::KeyPoint> &keyPoints,
                      cv::Mat &descriptors) const {
  _detector->detectAndCompute(image, cv::Mat(), keyPoints, descriptors);
  if (_sampleSize == 0) {
    subsample(keyPoints, descriptors);
  }
}

void Feature::subsample(std::vector<cv::KeyPoint> &keyPoints,
                        cv::Mat &descriptors) const {
  assert(keyPoints.size() == descriptors.rows);
  assert(_sampleSize > 0);

  std::vector<unsigned int> indices(keyPoints.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::random_shuffle(indices.begin(), indices.end());
  std::vector<cv::KeyPoint> subKeyPoints;
  cv::Mat subDescriptors;
  for (int i = 0; i < _sampleSize && i < indices.size(); i++) {
    subKeyPoints.emplace_back(keyPoints[i]);
    subDescriptors.push_back(descriptors.row(i));
  }

  keyPoints = subKeyPoints;
  descriptors = subDescriptors;
}
