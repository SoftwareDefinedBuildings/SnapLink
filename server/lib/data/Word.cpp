#include "Word.h"

Word::Word(int id, const cv::Mat &descriptor, int dbId,
           const std::vector<cv::Point3f> &points3)
    : _id(id), _descriptor(descriptor), _dbId(dbId), _points3(points3) {}

void Word::addPoints3(const std::vector<cv::Point3f> &points3) {
  _points3.insert(_points3.end(), points3.begin(), points3.end());
}

int Word::getId() const { return _id; }

const cv::Mat &Word::getDescriptor() const { return _descriptor; }

int Word::getDbId() const { return _dbId; }

const std::vector<cv::Point3f> &Word::getPoints3() const { return _points3; }
