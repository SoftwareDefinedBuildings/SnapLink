#include "Word.h"

Word::Word(int id, const cv::Mat &descriptor) :
    _id(id),
    _descriptor(descriptor)
{
}

int Word::getId() const
{
    return _id;
}

const cv::Mat &Word::getDescriptor() const
{
    return _descriptor;
}
