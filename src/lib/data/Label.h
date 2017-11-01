#pragma once

#include <opencv2/core/core.hpp>

class Label final {
public:
  explicit Label(int dbId, const cv::Point3f &point3,
                 const std::string &name);

  int getDbId() const;
  const cv::Point3f &getPoint3() const;
  const std::string &getName() const;

private:
  int _dbId;
  cv::Point3f _point3;
  std::string _name;
};
