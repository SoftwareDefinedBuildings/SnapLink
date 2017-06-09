#include "lib/util/Utility.h"
#include "lib/data/Image.h"
#include "lib/data/Transform.h"
#include "lib/data/FoundItem.h"
#include <pcl/common/transforms.h>
#include <rtabmap/core/util3d.h>
#include <stddef.h>
#include <sys/time.h>
#include <zbar.h> 
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>
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

bool Utility::qrExtract(const cv::Mat &im, std::vector<FoundItem> &results) {
  std::cout<<"Qr extracting\n";
  zbar::ImageScanner scanner;   
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);   
  uchar *raw = (uchar *)im.data;
  int width = im.cols;
  int height = im.rows;
  std::cout<<"Widith = "<<width<<" Height = "<<height<<std::endl;
  zbar::Image image(width, height, "Y800", raw, width * height);
  int n = scanner.scan(image); 
  std::cout<<"n is "<<n<<std::endl;
  for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  symbol != image.symbol_end(); ++symbol) {
    std::cout << "decoded " << symbol->get_type_name() << " symbol "<< symbol->get_data() << std::endl; 
    double x0 = symbol->get_location_x(0);
    double x1 = symbol->get_location_x(1);
    double x2 = symbol->get_location_x(2);
    double x3 = symbol->get_location_x(3);
    double y0 = symbol->get_location_y(0);
    double y1 = symbol->get_location_y(1);
    double y2 = symbol->get_location_y(2);
    double y3 = symbol->get_location_y(3);
    double meanX = (x0 + x1 + x2 + x3)/4;
    double meanY = (y0 + y1 + y2 + y3)/4;
    double width = (meanX - x0) > 0 ? (meanX - x0) : (x0 - meanX); 
    FoundItem item(symbol->get_data(), meanX, meanY, width);
    std::cout<<"Width is "<<width<<std::endl;
    std::cout<<"X is "<<meanX<<std::endl;
    std::cout<<"Y is "<<meanY<<std::endl;
    results.push_back(item);
  } 
  return results.size() > 0;
}
