#include <cassert>
#include "Label.h"

Label::Label(int dbId,
             int signatureId,
             cv::Point2f point2,
             cv::Point3f point3,
             std::string name):
    _dbId(dbId),
    _signatureId(signatureId),
    _point2(point2),
    _point3(point3),
    _name(name)
{
}

Label::~Label()
{
}

int Label::getDbId() const
{
    return _dbId;
}

int Label::getSignatureId() const
{
    return _signatureId;
}

cv::Point2f Label::getPoint2() const
{
    return _point2;
}

cv::Point3f Label::getPoint3() const
{
    return _point3;
}

std::string Label::getName() const
{
    return _name;
}
