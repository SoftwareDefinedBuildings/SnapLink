#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d.h>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <QCoreApplication>
#include <QDirIterator>
#include <QTextStream>
#include <fstream>
#include "util/Utility.h"
#include <iostream>
#include "stage/Visibility.h"
#include "event/LocationEvent.h"
#include "event/DetectionEvent.h"
#include "event/FailureEvent.h"
#include "data/PerfData.h"

Visibility::Visibility() :
    _httpServer(nullptr)
{
}

Visibility::~Visibility()
{
    _httpServer = nullptr;
}

void Visibility::setLabels(std::unique_ptr<Labels> &&labels)
{
    _labels = std::move(labels);
}

void Visibility::setHTTPServer(HTTPServer *httpServer)
{
    _httpServer = httpServer;
}

bool Visibility::event(QEvent *event)
{
    if (event->type() == LocationEvent::type())
    {
        LocationEvent *locEvent = static_cast<LocationEvent *>(event);
        std::unique_ptr<rtabmap::SensorData> sensorData = locEvent->takeSensorData();
        std::unique_ptr<rtabmap::Transform> pose = locEvent->takePose();
        std::unique_ptr<PerfData> perfData = locEvent->takePerfData();
        const void *session = locEvent->getSession();
        std::unique_ptr< std::vector<std::string> > names(new std::vector<std::string>());
        *names = process(locEvent->dbId(), *sensorData, *pose);
        if (!names->empty())
        {
            QCoreApplication::postEvent(_httpServer, new DetectionEvent(std::move(names), std::move(perfData), session));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(session));
        }
        return true;
    }
    return QObject::event(event);
}

std::vector<std::string> Visibility::process(int dbId, const rtabmap::SensorData &sensorData, const rtabmap::Transform &pose) const
{
    const std::list< std::unique_ptr<Label> > &labels = _labels->getLabels().at(dbId);
    std::vector<cv::Point3f> points;
    std::vector<std::string> names;
    for (auto & label : labels)
    {
        points.push_back(label->getPoint3());
        names.push_back(label->getName());
    }

    std::vector<std::string> results;

    UDEBUG("processing transform = %s", pose.prettyPrint().c_str());

    std::vector<cv::Point2f> planePoints;
    std::vector<std::string> visibleLabels;

    const rtabmap::CameraModel &model = sensorData.cameraModels()[0];
    cv::Mat K = model.K();
    rtabmap::Transform P = (pose * model.localTransform()).inverse();
    cv::Mat R = (cv::Mat_<double>(3, 3) <<
                 (double)P.r11(), (double)P.r12(), (double)P.r13(),
                 (double)P.r21(), (double)P.r22(), (double)P.r23(),
                 (double)P.r31(), (double)P.r32(), (double)P.r33());
    cv::Mat rvec(1, 3, CV_64FC1);
    cv::Rodrigues(R, rvec);
    cv::Mat tvec = (cv::Mat_<double>(1, 3) <<
                    (double)P.x(), (double)P.y(), (double)P.z());

    // do the projection
    cv::projectPoints(points, rvec, tvec, K, cv::Mat(), planePoints);

    // find points in the image
    int cols = sensorData.imageRaw().cols;
    int rows = sensorData.imageRaw().rows;
    std::map< std::string, std::vector<double> > distances;
    std::map< std::string, std::vector<cv::Point2f> > labelPoints;
    cv::Point2f center(cols / 2, rows / 2);

    for (unsigned int i = 0; i < points.size(); ++i)
    {
        //if (uIsInBounds(int(planePoints[i].x), 0, cols) &&
        //        uIsInBounds(int(planePoints[i].y), 0, rows))
        if (true)
        {
            if (Utility::isInFrontOfCamera(points[i], P))
            {
                std::string name = names[i];
                visibleLabels.push_back(name);
                float x, y, z, roll, pitch, yaw;
                pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                cv::Point3f cameraLoc(x, y, z);
                //double dist = cv::norm(points[i] - cameraLoc);
                double dist = cv::norm(planePoints[i] - center);
                distances[name].push_back(dist);
                labelPoints[name].push_back(planePoints[i]);
                UDEBUG("Find label %s at (%lf, %lf), image size=(%d,%d)", names[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
            }
            else
            {
                UDEBUG("Label %s invalid at (%lf, %lf) because it is from the back of the camera, image size=(%d,%d)", names[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
            }
        }
        else
        {
            UDEBUG("label %s invalid at (%lf, %lf), image size=(%d,%d)", names[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
        }
    }

    if (!distances.empty())
    {
        // find the label with minimum mean distance
        std::pair< std::string, std::vector<double> > minDist = *min_element(distances.begin(), distances.end(), CompareMeanDist());
        std::string minlabel = minDist.first;
        UINFO("Nearest label %s with mean distance %lf", minlabel.c_str(), CompareMeanDist::meanDist(minDist.second));
        results.push_back(minlabel);
    }
    else
    {
        UINFO("No label is qualified");
    }
    return results;
}

double CompareMeanDist::meanDist(const std::vector<double> &vec)
{
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    double mean = sum / vec.size();
    return mean;
}

bool CompareMeanDist::operator()(const PairType &left, const PairType &right) const
{
    return meanDist(left.second) < meanDist(right.second);
}

