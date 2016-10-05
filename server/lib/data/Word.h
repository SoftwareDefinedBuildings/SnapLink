#pragma once

#include <map>
#include <opencv2/core/core.hpp>

class Word {
public:
  Word(int id);

  /*
   * Add points and their descriptors in a database
   */
  void addPoints3(int dbId, const std::vector<cv::Point3f> &points3,
                  const cv::Mat &descriptors);

  int getId() const;
  const cv::Mat &getMeanDescriptor() const;
  const std::map<int, std::vector<cv::Point3f>> &getPoints3Map() const;
  const std::map<int, cv::Mat> &getDescriptorsByDb() const;

private:
  int _id;
  cv::Mat _meanDescriptor;
  cv::Mat _allDescriptors;
  std::map<int, std::vector<cv::Point3f>> _points3Map; // dbId : points3
  std::map<int, cv::Mat> _descriptorsByDb;             // dbId : descriptors
};
