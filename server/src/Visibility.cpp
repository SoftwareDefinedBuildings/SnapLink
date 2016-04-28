#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <QCoreApplication>
#include <QDirIterator>
#include <QTextStream>
#include <fstream>
#include <iostream>
#include <sqlite3.h>
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

bool Visibility::init(const std::string &dbfile)
{
    return processLabels(dbfile);
}

void Visibility::setMemory(const MemoryLoc *memory)
{
    _memory = memory;
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

bool Visibility::processLabels(const std::string &dbfile)
{
    sqlite3 *db = NULL;
    sqlite3_stmt *stmt = NULL;
    int rc;

    rc = sqlite3_open(dbfile.c_str(), &db);
    if (rc != SQLITE_OK)
    {
        UERROR("Could not open database %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return false;
    }

    std::string sql = "SELECT * from Labels";
    rc = sqlite3_prepare(db, sql.c_str(), -1, &stmt, NULL);
    if (rc != SQLITE_OK)
    {
        UERROR("Could not read database: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return false;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        std::string label(reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
        int imageId = sqlite3_column_int(stmt, 1);
        int x = sqlite3_column_int(stmt, 2);
        int y = sqlite3_column_int(stmt, 3);
        pcl::PointXYZ pWorld;
        if (getPoint3World(imageId, x, y, _memory, pWorld))
        {
            _points.push_back(cv::Point3f(pWorld.x, pWorld.y, pWorld.z));
            _labels.push_back(label);
            UINFO("Read point (%lf,%lf,%lf) with label %s", pWorld.x, pWorld.y, pWorld.z, label.c_str());
        }
    }

    sqlite3_finalize(stmt);

    sqlite3_close(db);

    return true;
}

// int Visibility::sqliteCallBack(int argc, char **argv, char **azColName)
// {
//     if (argc != 4 || argv == NULL)
//     {
//         return 1;
//     }
//     for (int i = 0; i < argc; i++)
//     {
//         if (argv[i] == NULL)
//         {
//             return 1;
//         }
//     }
//
//     std::string label(argv[0]);
//     int imageId = std::atoi(argv[1]);
//     int x = std::atoi(argv[2]);
//     int y = std::atoi(argv[3]);
//     pcl::PointXYZ pWorld;
//     if (getPoint3World(imageId, x, y, _memory, pWorld))
//     {
//         _points.push_back(cv::Point3f(pWorld.x, pWorld.y, pWorld.z));
//         _labels.push_back(label);
//         UDEBUG("Read point (%lf,%lf,%lf) with label %s", pWorld.x, pWorld.y, pWorld.z, label.c_str());
//     }
//
//     return 0;
// }

std::vector<std::string> *Visibility::process(const rtabmap::SensorData *data, const rtabmap::Transform &pose) const
{
    std::vector<std::string> *names = new std::vector<std::string>();
    if (_points.size() == 0)
    {
        return names;
    }

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
                std::string label = _labels[i];
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

// int Visibility::sqliteCallBack(void *param, int argc, char **argv, char **azColName)
// {
//     Visibility *vis = reinterpret_cast<Visibility *>(param);
//     return vis->sqliteCallback(argc, argv, azColName);
// }

bool Visibility::getPoint3World(int imageId, int x, int y, const MemoryLoc *memory, pcl::PointXYZ &pWorld)
{
    const rtabmap::Signature *s = memory->getSignature(imageId);
    if (s == NULL)
    {
        UWARN("Signature %d does not exist", imageId);
        return false;
    }
    const rtabmap::SensorData &data = s->sensorData();
    const rtabmap::CameraModel &cm = data.cameraModels()[0];
    bool smoothing = false;
    pcl::PointXYZ pLocal = rtabmap::util3d::projectDepthTo3D(data.depthRaw(), x, y, cm.cx(), cm.cy(), cm.fx(), cm.fy(), smoothing);
    if (std::isnan(pLocal.x) || std::isnan(pLocal.y) || std::isnan(pLocal.z))
    {
        UWARN("Depth value not valid");
        return false;
    }
    rtabmap::Transform poseWorld = memory->getOptimizedPose(imageId);
    if (poseWorld.isNull())
    {
        UWARN("Image pose is Null");
        return false;
    }
    poseWorld = poseWorld * cm.localTransform();
    pWorld = rtabmap::util3d::transformPoint(pLocal, poseWorld);
    return true;
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

