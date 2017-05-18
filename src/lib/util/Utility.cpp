#include "lib/util/Utility.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include <pcl/common/transforms.h>
#include <rtabmap/core/util3d.h>
#include <stddef.h>
#include <sys/time.h>

unsigned long long Utility::getTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

void Utility::showProgress(float progress) {
  assert(progress >= 0);
  assert(progress <= 1);
  const int barWidth = 70;

  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; i++) {
    if (i < pos) {
      std::cout << "=";
    } else if (i == pos) {
      std::cout << ">";
    } else {
      std::cout << " ";
    }
  }
  std::cout << "] " << static_cast<int>(progress * 100.0) << " %\r";
  std::cout.flush();
}

bool Utility::getPoint3World(const Image &image, const cv::Point2f &point2,
                             cv::Point3f &point3) {
  Transform pose = image.getPose();
  assert(!pose.isNull());

  const CameraModel &camera = image.getCameraModel();
  bool smoothing = false;
  pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(
      image.getDepth(), point2.x, point2.y, camera.cx(), camera.cy(),
      camera.fx(), camera.fy(), smoothing);
  if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z)) {
    // std::cerr << "Depth value not valid" << std::endl;
    return false;
  }
  pcl::PointXYZ point3PCL = pcl::transformPoint(pLocal, pose.toEigen3f());
  point3 = cv::Point3f(point3PCL.x, point3PCL.y, point3PCL.z);
  return true;
}

bool Utility::compareCVPoint2f(cv::Point2f p1, cv::Point2f p2) {
  return ((p1.x < p2.x) || ((p1.x == p2.x) && (p1.y < p2.y)));
}

bool Utility::isInFrontOfCamera(const cv::Point3f &point,
                                const Transform &pose) {
  pcl::PointXYZ pointPCL(point.x, point.y, point.z);
  pcl::PointXYZ newPointPCL = pcl::transformPoint(pointPCL, pose.toEigen3f());
  return newPointPCL.z > 0;
}
