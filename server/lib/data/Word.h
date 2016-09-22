#pragma once

#include <map>
#include <opencv2/core/core.hpp>

class Word {
public:
  Word(int id, const cv::Mat &descriptor,
       const std::vector<cv::Point3f> &points3);

  int getId() const;
  const cv::Mat &getDescriptor() const;
  const std::vector<cv::Point3f> &getPoints3() const;

private:
  int _id;
  cv::Mat _descriptor;
  std::vector<cv::Point3f> _points3;
};
