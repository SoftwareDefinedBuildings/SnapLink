#pragma once

#include <opencv2/core/core.hpp>

class Label
{
public:
    Label(int dbId,
          int signatureId,
          cv::Point2f point2,
          cv::Point3f point3,
          std::string name);
    virtual ~Label();

    int getDbId() const;
    int getSignatureId() const;
    cv::Point2f getPoint2() const;
    cv::Point3f getPoint3() const;
    std::string getName() const;

private:
    int _dbId;
    int _signatureId;
    cv::Point2f _point2;
    cv::Point3f _point3;
    std::string _name;
};

