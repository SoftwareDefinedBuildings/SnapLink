#include "lib/data/FoundItem.h"

FoundItem::FoundItem() = default;

FoundItem::FoundItem(const std::string &name, double x, double y, double size,
                     double width, double height) {
  _name = name;
  _x = x;
  _y = y;
  _size = size;
  _width = width;
  _height = height;
}

const std::string &FoundItem::name() const { return _name; }

double FoundItem::x() const { return _x; }
double FoundItem::y() const { return _y; }
double FoundItem::size() const { return _size; }
double FoundItem::width() const { return _width; }
double FoundItem::height() const { return _height; }

void FoundItem::setX(double x) { _x = x; }
void FoundItem::setY(double y) { _y = y; }
void FoundItem::setSize(double size) { _size = size; }
void FoundItem::setWidth(double width) { _width = width; }
void FoundItem::setHeight(double height) { _height = height; }
