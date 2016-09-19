#include "Word.h"

Word::Word(int id, const cv::Mat &descriptor, const std::map<int, int> &references)
    : _id(id), _descriptor(descriptor), _references(references) {}

int Word::getId() const { return _id; }

const cv::Mat &Word::getDescriptor() const { return _descriptor; }
  
const std::map<int, int> &Word::getReferences() const { return _references; }
