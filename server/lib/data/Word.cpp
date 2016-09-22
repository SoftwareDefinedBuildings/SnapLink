#include "Word.h"

Word::Word(int id, const cv::Mat &descriptor,
           const std::vector<cv::Point3f> &points3)
    : _id(id), _descriptor(descriptor), _points3(points3) {}

int Word::getId() const { return _id; }

const cv::Mat &Word::getDescriptor() const { return _descriptor; }

const std::vector<cv::Point3f> &Word::getPoints3() const { return _points3; }
