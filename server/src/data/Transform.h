#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

class Transform
{
public:
    Transform();

    // rotation matrix r## and origin o##
    Transform(float r11, float r12, float r13, float t14,
              float r21, float r22, float r23, float t24,
              float r31, float r32, float r33, float t34);

    float r11() const {return data()[0];}
    float r12() const {return data()[1];}
    float r13() const {return data()[2];}
    float r21() const {return data()[4];}
    float r22() const {return data()[5];}
    float r23() const {return data()[6];}
    float r31() const {return data()[8];}
    float r32() const {return data()[9];}
    float r33() const {return data()[10];}
    float x() {return data()[3];}
    float y() {return data()[7];}
    float z() {return data()[11];}

    bool isNull() const;

    const float * data() const {return (const float *)_data.data;}
    int size() const {return 12;}

    Transform inverse() const;

    std::string prettyPrint() const;

    Transform operator*(const Transform & t) const;
    Transform &operator*=(const Transform & t);
    bool operator==(const Transform & t) const;
    bool operator!=(const Transform & t) const;

    Eigen::Matrix4f toEigen4f() const;
    Eigen::Affine3f toEigen3f() const;

public:
    static Transform fromEigen4f(const Eigen::Matrix4f & matrix);
    static Transform fromEigen4d(const Eigen::Matrix4d & matrix);

private:
    cv::Mat _data;
};
