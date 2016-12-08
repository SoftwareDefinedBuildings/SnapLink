#include "algo/Feature.h"

Feature::Feature() {
  int minHessian = 400;
  _detector = cv::xfeatures2d::SURF::create(minHessian);
}

void Feature::extract(const cv::Mat &image,
                      std::vector<cv::KeyPoint> &keyPoints,
                      cv::Mat &descriptors) const {
  _detector->detectAndCompute(image, cv::Mat(), keyPoints, descriptors);
}
