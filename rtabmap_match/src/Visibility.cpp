#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <QCoreApplication>
#include <QDirIterator>
#include <QTextStream>
#include <fstream>
#include <iostream>
#include "Utility.h"
#include "Visibility.h"
#include "LocationEvent.h"
#include "DetectionEvent.h"
#include "FailureEvent.h"

Visibility::Visibility() :
    _httpServer(NULL)
{
}

Visibility::~Visibility()
{
    _httpServer = NULL;
}

bool Visibility::init(const std::string &dir, const MemoryLoc *memory)
{
    return readLabels(dir);
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
        std::vector<std::string> *names = process(locEvent->sensorData(), locEvent->pose());
        if (names != NULL)
        {
            QCoreApplication::postEvent(_httpServer, new DetectionEvent(names, locEvent->conInfo()));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(locEvent->conInfo()));
        }
        delete locEvent->sensorData();
        return true;
    }
    return QObject::event(event);
}

bool Visibility::readLabels(const std::string &dir)
{
    QString filter = QString::fromStdString("*.txt");
    QDirIterator it(QString::fromStdString(dir), QStringList() << filter, QDir::Files, QDirIterator::NoIteratorFlags);
    while (it.hasNext())
    {
        QString fileName = it.next();

        // read labels from file
        QFile file(fileName);
        if (!file.open(QFile::ReadOnly | QFile::Text))
        {
            return false;
        }

        std::string label = QFileInfo(fileName).baseName().toStdString();
        QTextStream in(&file);
        while (!in.atEnd())
        {
            QString line = in.readLine();
            QStringList list = line.split(",");
            if (list.size() != 3)
            {
                return false;
            }
            double imageId = list.at(0).toDouble();
            double x = list.at(1).toDouble();
            double y = list.at(2).toDouble();
            _points.push_back(cv::Point3f(imageId, x, y));
            _labels.push_back(label);
            UDEBUG("Read point (%lf,%lf,%lf) with label %s", imageId, x, y, label.c_str());
        }
    }

    return true;
}

std::vector<std::string> *Visibility::process(const rtabmap::SensorData *data, const rtabmap::Transform &pose)
{
    UDEBUG("processing transform = %s", pose.prettyPrint().c_str());

    std::vector<cv::Point2f> planePoints;
    std::vector<std::string> visibleLabels;

    const rtabmap::CameraModel &model = data->cameraModels()[0];
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
    cv::projectPoints(_points, rvec, tvec, K, cv::Mat(), planePoints);

    // find points in the image
    int cols = data->imageRaw().cols;
    int rows = data->imageRaw().rows;
    std::map< std::string, std::vector<double> > distances;
    std::map< std::string, std::vector<cv::Point2f> > labelPoints;
    cv::Point2f center(cols / 2, rows / 2);

    for (unsigned int i = 0; i < _points.size(); ++i)
    {
        //if (uIsInBounds(int(planePoints[i].x), 0, cols) &&
        //        uIsInBounds(int(planePoints[i].y), 0, rows))
        if (true)
        {
            if (Utility::isInFrontOfCamera(_points[i], P))
            {
                std::string &label = _labels[i];
                visibleLabels.push_back(label);
                float x, y, z, roll, pitch, yaw;
                pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                cv::Point3f cameraLoc(x, y, z);
                //double dist = cv::norm(_points[i] - cameraLoc);
                double dist = cv::norm(planePoints[i] - center);
                distances[label].push_back(dist);
                labelPoints[label].push_back(planePoints[i]);
                UDEBUG("Find label %s at (%lf, %lf), image size=(%d,%d)", _labels[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
            }
            else
            {
                UDEBUG("Label %s invalid at (%lf, %lf) because it is from the back of the camera, image size=(%d,%d)", _labels[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
            }
        }
        else
        {
            UDEBUG("label %s invalid at (%lf, %lf), image size=(%d,%d)", _labels[i].c_str(), planePoints[i].x, planePoints[i].y, cols, rows);
        }
    }

    std::vector<std::string> *names = new std::vector<std::string>();
    if (!distances.empty())
    {
        // find the label with minimum mean distance
        std::pair< std::string, std::vector<double> > minDist = *min_element(distances.begin(), distances.end(), CompareMeanDist());
        std::string minlabel = minDist.first;
        UINFO("Nearest label %s with mean distance %lf", minlabel.c_str(), CompareMeanDist::meanDist(minDist.second));
        names->push_back(minlabel);
    }
    else
    {
        UINFO("No label is qualified");
    }
    return names;
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
