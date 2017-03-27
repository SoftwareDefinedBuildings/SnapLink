#pragma once

#include <opencv2/core/core.hpp>

class Label final {
public:
  explicit Label(int roomId, const cv::Point3f &point3,
                 const std::string &name);

  int getRoomId() const;
  const cv::Point3f &getPoint3() const;
  const std::string &getName() const;

private:
  int _roomId;
  cv::Point3f _point3;
  std::string _name;
};
