#pragma once
#include <string>
class FoundItem final {
public:
  explicit FoundItem();
  
  explicit FoundItem(const std::string &name, double x, double y);

  const std::string &name() const;
  double x() const;
  double y() const;

private:
  std::string _name;
  double _x;
  double _y;  
};
