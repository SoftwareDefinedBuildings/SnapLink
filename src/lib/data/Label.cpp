#include "lib/data/Label.h"
#include <cassert>

Label::Label(int dbId, cv::Point3f point3, std::string name)
    : _dbId(dbId), _point3(point3), _name(std::move(name)) {}

int Label::getDbId() const { return _dbId; }

cv::Point3f Label::getPoint3() const { return _point3; }

std::string Label::getName() const { return _name; }
