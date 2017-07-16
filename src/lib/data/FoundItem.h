#pragma once
#include <string>
class FoundItem final {
public:
  explicit FoundItem();
  
  explicit FoundItem(const std::string &name, double x, double y, double size, double width, double height);

  const std::string &name() const;
  double x() const;
  double y() const;
  double size() const;
  double width() const;
  double height() const;
  void setX(double x);
  void setY(double y);
  void setSize(double size);
  void setWidth(double width);
  void setHeight(double height);
private:
  std::string _name;
  double _x;
  double _y;  
  double _size;
  double _width;
  double _height;
};
