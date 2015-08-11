/*
 * Author: Kaifei Chen <kaifei@berkeley.edu>
 */

#include <opencv2/core/core.hpp>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d_transforms.h>

#include <pcl/point_types.h>

#include <stdio.h>

#include "Visibility.h"

namespace rtabmap {
    
double CompareMeanDist::meanDist(const std::vector<double> & vec)
{
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum / vec.size();
    return mean;
}

bool CompareMeanDist::operator()(const PairType & left, const PairType & right) const
{
    return meanDist(left.second) < meanDist(right.second);
}

Visibility::Visibility(const CameraModel & model):
    _model(model)
{
    UDEBUG("");
    resultFile.open("result.txt");
    resultFileForPlot.open("result_matrix.txt");
}

Visibility::~Visibility()
{
    UDEBUG("");
    resultFile.close();
    resultFileForPlot.close();
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
    resultFileForPlot << "Image Name";
    while (true) {
        fileName = dir.getNextFileName();
        if(!fileName.size())
        {
            break;
        }
        fullPath = path + fileName;

        // read labels from file
        FILE * pFile = fopen(fullPath.c_str(), "r");
        int count = 0;
        while(fscanf(pFile, "%lf,%lf,%lf", &x, &y, &z) != EOF) {
            _points.push_back(cv::Point3f(x,y,z));
            label = uSplit(fileName, '.').front();
            _labels.push_back(label);
            resultFileForPlot << ", " << label << " " << count;
            count++;
            UDEBUG("Read point (%lf,%lf,%lf) with label %s", x, y, z, label.c_str());
        }
    }
    resultFileForPlot << std::endl;
}

void Visibility::process(const SensorData & data, const Transform & pose)
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

    //std::cout << "K: " << K << std::endl;
    //std::cout << "rvec: " << rvec << std::endl;
    //std::cout << "tvec: " << tvec << std::endl;
  
    // do the projection 
    cv::projectPoints(_points, rvec, tvec, K, cv::Mat(), planePoints);

    // find points in the image
    int cols = data.imageRaw().cols;
    int rows = data.imageRaw().rows;
    std::map< std::string, std::vector<double> > distances;
    std::map< std::string, std::vector<cv::Point2f> > labelPoints;
    
    for(unsigned int i = 0; i < _points.size(); ++i)
    {
        if(uIsInBounds(int(planePoints[i].x), 0, cols) &&
           uIsInBounds(int(planePoints[i].y), 0, rows))
        {
            if (isInFrontOfCamera(_points[i], P)) {
                std::string & label = _labels[i];
                visibleLabels.push_back(label);
                float x,y,z,roll,pitch,yaw;
                pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                cv::Point3f cameraLoc(x, y, z);
                double dist = cv::norm(_points[i] - cameraLoc);
                distances[label].push_back(dist);
                labelPoints[label].push_back(planePoints[i]);
                UINFO("Find label %s at (%lf, %lf), image size=(%d,%d)", _labels[i].c_str(),
                       planePoints[i].x, planePoints[i].y, cols, rows);
                resultFileForPlot  << ", 1";
            }
            else
            {
                UINFO("Label %s invalid at (%lf, %lf) because it is from the back of the camera, image size=(%d,%d)", _labels[i].c_str(),
                       planePoints[i].x, planePoints[i].y, cols, rows);
                resultFileForPlot  << ", 0";
            }
        }
        else 
        {
            UINFO("label %s invalid at (%lf, %lf), image size=(%d,%d)", _labels[i].c_str(), 
                    planePoints[i].x, planePoints[i].y, cols, rows);
            resultFileForPlot  << ", 0";
        }
    }
    resultFileForPlot << std::endl;
    resultFileForPlot.flush();

    if (distances.size() == 0) {
        return;
    }

    // find the label with minimum mean distance
    std::pair< std::string, std::vector<double> > minDist = *min_element(distances.begin(), distances.end(), CompareMeanDist());
    std::string minlabel = minDist.first;
    UINFO("Nearest label %s with mean disntace %lf", minlabel.c_str(), CompareMeanDist::meanDist(minDist.second));

    //write result to file
    resultFile << minlabel << std::endl;
    for(unsigned int i = 0; i < labelPoints[minlabel].size(); ++i)
    {
        cv::Point2f p = labelPoints[minlabel][i];
        resultFile << p.x << "," << p.y << std::endl;
    }
    resultFile.flush();
}
    
bool Visibility::isInFrontOfCamera(const cv::Point3f & point, const Transform & P)
{
    pcl::PointXYZ pointPCL(point.x, point.y, point.z);
    pcl::PointXYZ newPointPCL = util3d::transformPoint(pointPCL, P);
    return newPointPCL.z > 0;
}

} // namespace rtabmap
