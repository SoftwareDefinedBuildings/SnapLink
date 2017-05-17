#pragma once

#include <map>
#include <opencv2/core/core.hpp>

class Word final {
public:
  explicit Word(int id);

  /*
   * Add points and their descriptors in a database
   */
  void addPoint3(int roomId, const cv::Point3f &point3, cv::Mat descriptor);

  int getId() const;
  const cv::Mat &getMeanDescriptor() const;
  const std::map<int, std::vector<cv::Point3f>> &getPoints3Map() const;
  const std::map<int, cv::Mat> &getDescriptorsByDb() const;

private:
  int _id;
  cv::Mat _meanDescriptor;
  cv::Mat _allDescriptors;
  std::map<int, std::vector<cv::Point3f>> _points3Map; // roomId : points3
  std::map<int, cv::Mat> _roomDescriptors;             // roomId : descriptors
};
