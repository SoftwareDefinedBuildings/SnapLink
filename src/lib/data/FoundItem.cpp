#include "lib/data/FoundItem.h"

FoundItem::FoundItem() = default;

FoundItem::FoundItem(const std::string &name, double x, double y) {
  _name = name;
  _x = x;
  _y = y;
}

const std::string &FoundItem::name() const { return _name; }

double FoundItem::x() const { return _x; }
double FoundItem::y() const { return _y; }
