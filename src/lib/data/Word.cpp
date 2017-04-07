#include "lib/data/Word.h"
#include <cassert>

Word::Word(int id) : _id(id), _dataChanged(false) {}

void Word::addPoint3(int roomId, const cv::Point3f &point3,
                     cv::Mat descriptor) {
  _points3Map[roomId].emplace(points3);
  _allDescriptors.push_back(descriptor);
  _roomDescriptors[roomId].push_back(descriptor);
  _dataChanged = true;
}

void Word::addPoints3(const std::vector<int> &roomIds,
                      const std::vector<cv::Point3f> &points3,
                      const cv::Mat &descriptors) {
  assert(roomIds.size() == descriptors.rows);
  assert(points3.size() == descriptors.rows);
  for (int i = 0; i < roomIds.size(); i++) {
    addPoint3(roomIds.at(i), points3.at(i), descriptors.row(i));
  }
}

int Word::getId() const { return _id; }

const cv::Mat &Word::getMeanDescriptor() const {
  if (_dataChanged) {
    int axis = 0;
    cv::reduce(_allDescriptors, _meanDescriptor, axis, CV_REDUCE_AVG);
    _dataChanged = false;
  }
  return _meanDescriptor;
}

const std::map<int, std::vector<cv::Point3f>> &Word::getPoints3Map() const {
  return _points3Map;
}

const std::map<int, cv::Mat> &Word::getDescriptorsByDb() const {
  return _roomDescriptors;
}
