#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

class Transform {
public:
  Transform();

  Transform(float r11, float r12, float r13, float t14, //
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

  const float *data() const;
  int size() const;

  Transform rotation() const;

  std::string prettyPrint() const;

  Transform operator*(const Transform &t) const;
  Transform &operator*=(const Transform &t);
  bool operator==(const Transform &t) const;
  bool operator!=(const Transform &t) const;

  Eigen::Matrix4f toEigen4f() const;
  Eigen::Affine3f toEigen3f() const;

public:
  static Transform fromEigen4f(const Eigen::Matrix4f &matrix);

private:
  cv::Mat _data;
};
