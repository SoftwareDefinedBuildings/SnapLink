#include "lib/data/FoundItem.h"

FoundItem::FoundItem() = default;

FoundItem::FoundItem(const std::string &name, double x, double y, double width) {
  _name = name;
  _x = x;
  _y = y;
  _width = width;
}

const std::string &FoundItem::name() const { return _name; }

double FoundItem::x() const { return _x; }
double FoundItem::y() const { return _y; }
double FoundItem::width() const { return _width; }
