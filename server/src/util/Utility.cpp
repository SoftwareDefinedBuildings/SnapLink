#include <rtabmap/core/util3d_transforms.h>

#include "util/Utility.h"

bool Utility::compareCVPoint2f(cv::Point2f p1, cv::Point2f p2)
{
    return ((p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y)));
}

bool Utility::isInFrontOfCamera(const cv::Point3f &point, const Transform &pose)
{
    pcl::PointXYZ pointPCL(point.x, point.y, point.z);
    rtabmap::Transform oldPose = rtabmap::Transform::fromEigen4f(pose.toEigen4f());
    pcl::PointXYZ newPointPCL = rtabmap::util3d::transformPoint(pointPCL, oldPose);
    return newPointPCL.z > 0;
}
