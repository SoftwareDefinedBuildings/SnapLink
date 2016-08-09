#include <pcl/common/eigen.h>
#include "data/Transform.h"

Transform::Transform() = default;

Transform::Transform(
        float r11, float r12, float r13, float t14,
        float r21, float r22, float r23, float t24,
        float r31, float r32, float r33, float t34)
{
    _data = (cv::Mat_<float>(3,4) <<
            r11, r12, r13, t14,
            r21, r22, r23, t24,
            r31, r32, r33, t34);
}

bool Transform::isNull() const
{
    if (_data.empty())
    {
        return true;
    }
    
    for (int i = 0; i < 12; i++)
    {
        if (data()[i] == 0.0f || std::isnan(data()[i]))
        {
            return true;
        }
    }

    return false;
}

Transform Transform::inverse() const
{
    return fromEigen4f(toEigen4f().inverse());
}

std::string Transform::prettyPrint() const
{
    if(this->isNull())
    {
        return "xyz=[null] rpy=[null]";
    }
    else
    {
        float x,y,z,roll,pitch,yaw;
        pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);
        std::ostringstream ss;
        ss << "xyz=" << x << "," << y << "," << z << " rpy=" << roll << "," << pitch << "," << yaw;
        return ss.str();;
    }
}

Transform Transform::operator*(const Transform & t) const
{
    return fromEigen4f(toEigen4f()*t.toEigen4f());
}

Transform & Transform::operator*=(const Transform & t)
{
    *this = *this * t;
    return *this;
}

bool Transform::operator==(const Transform & t) const
{
    return memcmp(_data.data, t._data.data, _data.total() * sizeof(float)) == 0;
}

bool Transform::operator!=(const Transform & t) const
{
    return !(*this == t);
}

Eigen::Matrix4f Transform::toEigen4f() const
{
    Eigen::Matrix4f m;
    m << data()[0], data()[1], data()[2], data()[3],
         data()[4], data()[5], data()[6], data()[7],
         data()[8], data()[9], data()[10], data()[11],
         0,0,0,1;
    return m;
}

Eigen::Affine3f Transform::toEigen3f() const
{
    return Eigen::Affine3f(toEigen4f());
}

Transform Transform::fromEigen4f(const Eigen::Matrix4f & matrix)
{
    return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
                     matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
                     matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
Transform Transform::fromEigen4d(const Eigen::Matrix4d & matrix)
{
    return Transform(matrix(0,0), matrix(0,1), matrix(0,2), matrix(0,3),
                     matrix(1,0), matrix(1,1), matrix(1,2), matrix(1,3),
                     matrix(2,0), matrix(2,1), matrix(2,2), matrix(2,3));
}
