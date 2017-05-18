#include "lib/data/Word.h"
#include <cassert>

Word::Word(int id) : _id(id), _newData(false) {}

void Word::addPoint3(int roomId, const cv::Point3f &point3,
                     cv::Mat descriptor) {
  // roomId will be added if not exists
  _points3Map[roomId].emplace_back(point3);
  _roomDescriptors[roomId].push_back(descriptor);
  _allDescriptors.push_back(descriptor);

  _newData = true;
}

int Word::getId() const { return _id; }

const cv::Mat &Word::getMeanDescriptor() {
  if (_newData) {
    int axis = 0;
    cv::reduce(_allDescriptors, _meanDescriptor, axis, CV_REDUCE_AVG);
    _newData = false;
  }
  return _meanDescriptor;
}

const cv::Mat &Word::getMeanDescriptor() const { return _meanDescriptor; }

const std::map<int, std::vector<cv::Point3f>> &Word::getPoints3Map() const {
  return _points3Map;
}

const std::map<int, cv::Mat> &Word::getDescriptorsByDb() const {
  return _roomDescriptors;
}
