/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include "opencv2/core/core.hpp"

#include "rtabmap/utilite/ULogger.h"
#include "Visibility.h"

namespace rtabmap {

Visibility::Visibility(const CameraModel & model):
    _model(model)
{
    UDEBUG("");
}

Visibility::~Visibility()
{
    UDEBUG("");
}

bool Visibility::init(const std::string & cloudFile, const std::string & labelFolder)
{
    return true;
}

void Visibility::process(SensorData data, Transform pose)
{
    UDEBUG("processing transform = %s", pose.prettyPrint().c_str());

    // Read test points
    std::vector<cv::Point2f> imagePoints = Generate2DPoints();
    std::vector<cv::Point3f> objectPoints = Generate3DPoints();
    std::vector<cv::Point2f> projectedPoints;

    cv::Mat K = _model.K();
    Transform P = (pose * _model.localTransform()).inverse();
    cv::Mat R = (cv::Mat_<double>(3,3) <<
            (double)P.r11(), (double)P.r12(), (double)P.r13(),
            (double)P.r21(), (double)P.r22(), (double)P.r23(),
            (double)P.r31(), (double)P.r32(), (double)P.r33());
    cv::Mat rvec(1, 3, CV_64FC1);
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(1,3) << 
            (double)P.x(), (double)P.y(), (double)P.z());

    std::cout << "K: " << K << std::endl;
    std::cout << "rvec: " << rvec << std::endl;
    std::cout << "tvec: " << tvec << std::endl;
    
    cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), projectedPoints);

    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
       std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }

}


std::vector<cv::Point2f> Visibility::Generate2DPoints()
{
    std::vector<cv::Point2f> points;

    double x,y;

    x=282;y=274;
    points.push_back(cv::Point2f(x,y));

    x=397;y=227;
    points.push_back(cv::Point2f(x,y));

    x=577;y=271;
    points.push_back(cv::Point2f(x,y));

    x=462;y=318;
    points.push_back(cv::Point2f(x,y));

    x=270;y=479;
    points.push_back(cv::Point2f(x,y));

    x=450;y=523;
    points.push_back(cv::Point2f(x,y));

    x=566;y=475;
    points.push_back(cv::Point2f(x,y));
    /*
    for(unsigned int i = 0; i < points.size(); ++i)
      {
      std::cout << points[i] << std::endl;
      }
    */
    return points;
}


std::vector<cv::Point3f> Visibility::Generate3DPoints()
{
    std::vector<cv::Point3f> points;
  
    double x,y,z;
  
    x=.5;y=.5;z=-.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=.5;y=.5;z=.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=-.5;y=.5;z=.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=-.5;y=.5;z=-.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=.5;y=-.5;z=-.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=-.5;y=-.5;z=-.5;
    points.push_back(cv::Point3f(x,y,z));
  
    x=-.5;y=-.5;z=.5;
    points.push_back(cv::Point3f(x,y,z));
  
    /*
    for(unsigned int i = 0; i < points.size(); ++i)
      {
      std::cout << points[i] << std::endl;
      }
    */
    return points;
}

} // namespace rtabmap
