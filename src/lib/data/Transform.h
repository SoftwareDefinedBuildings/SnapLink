#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/viz.hpp>
#include <string>
#include <vector>

class Transform final {
public:
  explicit Transform();

  explicit Transform(float r11, float r12, float r13, float t14, //
                     float r21, float r22, float r23, float t24, //
                     float r31, float r32, float r33, float t34);

  float r11() const;
  float r12() const;
  float r13() const;
  float r21() const;
  float r22() const;
  float r23() const;
  float r31() const;
  float r32() const;
  float r33() const;
  float x() const;
  float y() const;
  float z() const;

  bool isNull() const;

  int size() const;

  Transform rotation() const;
  Transform translation() const;
  Transform inverse() const;

  Transform operator*(const Transform &t) const;
  Transform &operator*=(const Transform &t);
  bool operator==(const Transform &t) const;
  bool operator!=(const Transform &t) const;
  friend std::ostream &operator<<(std::ostream &out, const Transform &t);
  friend std::istream &operator>>(std::istream &in, Transform &t);

  Eigen::Matrix4f toEigen4f() const;
  Eigen::Affine3f toEigen3f() const;
  cv::Affine3f toAffine3f() const;


private:
  const float *data() const;
  static Transform fromEigen4f(const Eigen::Matrix4f &matrix);

private:
  cv::Mat _data;
};
