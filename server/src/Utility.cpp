#include <rtabmap/core/util3d_transforms.h>

#include "Utility.h"

bool Utility::compareCVPoint2f(cv::Point2f p1, cv::Point2f p2)
{
    return ((p1.x < p2.x) || (p1.x == p2.x) && (p1.y < p2.y));
}

bool Utility::isInFrontOfCamera(const cv::Point3f &point, const rtabmap::Transform &P)
{
    pcl::PointXYZ pointPCL(point.x, point.y, point.z);
    pcl::PointXYZ newPointPCL = rtabmap::util3d::transformPoint(pointPCL, P);
    return newPointPCL.z > 0;
}
