#include "util/Utility.h"
#include <pcl/common/transforms.h>

bool Utility::compareCVPoint2f(cv::Point2f p1, cv::Point2f p2) {
  return ((p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y)));
}

bool Utility::isInFrontOfCamera(const cv::Point3f &point,
                                const Transform &pose) {
  pcl::PointXYZ pointPCL(point.x, point.y, point.z);
  pcl::PointXYZ newPointPCL = pcl::transformPoint(pointPCL, pose.toEigen3f());
  return newPointPCL.z > 0;
}
