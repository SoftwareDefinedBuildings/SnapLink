#pragma once

#include <opencv2/core/core.hpp>

class Label final {
public:
  explicit Label(int roomId, cv::Point3f point3, std::string name);

  int getRoomId() const;
  cv::Point3f getPoint3() const;
  std::string getName() const;

private:
  int _roomId;
  cv::Point3f _point3;
  std::string _name;
};
