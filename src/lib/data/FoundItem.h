#pragma once
#include <string>
class FoundItem final {
public:
  explicit FoundItem();
  
  explicit FoundItem(const std::string &name, double x, double y, double width);

  const std::string &name() const;
  double x() const;
  double y() const;
  double width() const;
private:
  std::string _name;
  double _x;
  double _y;  
  double _width;
};
