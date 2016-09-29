#pragma once

#include <map>
#include <opencv2/core/core.hpp>

class Word {
public:
  Word(int id, const cv::Mat &descriptor, int dbId,
       const std::vector<cv::Point3f> &points3);

  int getId() const;
  const cv::Mat &getDescriptor() const;
  int getDbId() const;
  const std::vector<cv::Point3f> &getPoints3() const;

private:
  int _id;
  cv::Mat _descriptor;
  int _dbId;
  std::vector<cv::Point3f> _points3;
};
