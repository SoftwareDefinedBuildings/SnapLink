/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include <opencv2/core/core.hpp>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>


#include <stdio.h>

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

bool Visibility::init(const std::string & labelFolder)
{
    return readLabels(labelFolder);
}
    
bool Visibility::readLabels(const std::string & labelFolder)
{
    std::string path = labelFolder;
    UDirectory dir(path, "txt");

    if(path[path.size()-1] != '\\' && path[path.size()-1] != '/')
    {
        path.append("/");
    }
    if(!dir.isValid())
    {
        ULOGGER_ERROR("path is not valid \"%s\"", path.c_str());
        return false;
    }
    else if(dir.getFileNames().size() == 0)
    {
        UWARN("path is empty \"%s\"", path.c_str());
        return false;
    }
    else
    {
        UINFO("path=%s number of label files=%d", path.c_str(), dir.getFileNames().size());
    }

    std::string fileName;
    std::string fullPath;
    double x, y, z;
    std::string label;
    while (true) {
        fileName = dir.getNextFileName();
        if(!fileName.size())
        {
            break;
        }
        fullPath = path + fileName;

        // read labels from file
        FILE * pFile = fopen(fullPath.c_str(), "r");
        while(fscanf(pFile, "%lf,%lf,%lf", &x, &y, &z) != EOF) {
            _points.push_back(cv::Point3f(x,y,z));
            label = uSplit(fileName, '.').front();
            _labels.push_back(label);
            UDEBUG("Read point (%lf,%lf,%lf) with label %s", x, y, z, label.c_str());
        }
    }
}

void Visibility::process(SensorData data, Transform pose)
{
    UDEBUG("processing transform = %s", pose.prettyPrint().c_str());

    std::vector<cv::Point2f> planePoints;
    std::vector<std::string> visibleLabels;

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
   
    // do the projection 
    cv::projectPoints(_points, rvec, tvec, K, cv::Mat(), planePoints);

    // find points in the image
    int cols = data.imageRaw().cols;
    int rows = data.imageRaw().rows;
    for(unsigned int i = 0; i < _points.size(); ++i)
    {
        if(uIsInBounds(int(planePoints[i].x), 0, cols) &&
           uIsInBounds(int(planePoints[i].y), 0, rows))
        {
            visibleLabels.push_back(_labels[i]);
            UDEBUG("Find label %s", _labels[i].c_str());
        }
        else 
        {
            UDEBUG("label %s invalid at (%lf, %lf), image size=(%d,%d)", _labels[i].c_str(), 
                    planePoints[i].x, planePoints[i].y, cols, rows);
        }
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
