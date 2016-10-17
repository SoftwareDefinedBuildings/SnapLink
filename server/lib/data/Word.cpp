#include "Word.h"
#include <cassert>

Word::Word(int id) : _id(id) {}

void Word::addPoints3(int dbId, const std::vector<cv::Point3f> &points3,
                      const cv::Mat &descriptors) {
  assert(points3.size() == descriptors.rows);
  _points3Map[dbId].insert(_points3Map[dbId].end(), points3.begin(),
                           points3.end());
  for (int i = 0; i < descriptors.rows; i++) {
    cv::Mat row = descriptors.row(i).clone();
    _allDescriptors.push_back(row);
    _descriptorsByDb[dbId].push_back(row);
  }
  cv::reduce(_allDescriptors, _meanDescriptor, 0, CV_REDUCE_AVG);
}

int Word::getId() const { return _id; }

const cv::Mat &Word::getMeanDescriptor() const { return _meanDescriptor; }

const std::map<int, std::vector<cv::Point3f>> &Word::getPoints3Map() const {
  return _points3Map;
}

const std::map<int, cv::Mat> &Word::getDescriptorsByDb() const {
  return _descriptorsByDb;
}
