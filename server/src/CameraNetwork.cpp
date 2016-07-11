#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>
#include <QCoreApplication>

#include "CameraNetwork.h"
#include "NetworkEvent.h"
#include "ImageEvent.h"
#include "FailureEvent.h"

CameraNetwork::CameraNetwork() :
    _feature(NULL),
    _httpServer(NULL)
{
}

CameraNetwork::~CameraNetwork()
{
    _feature = NULL;
    _httpServer = NULL;
}

void CameraNetwork::setFeatureExtraction(FeatureExtraction *feature)
{
    _feature = feature;
}

void CameraNetwork::setHTTPServer(HTTPServer *httpServer)
{
    _httpServer = httpServer;
}

bool CameraNetwork::event(QEvent *event)
{
    if (event->type() == NetworkEvent::type())
    {
        NetworkEvent *networkEvent = static_cast<NetworkEvent *>(event);

        std::vector<unsigned char> *data = &networkEvent->conInfo()->data;
        int width = networkEvent->conInfo()->width;
        int height = networkEvent->conInfo()->height;
        double fx = networkEvent->conInfo()->fx;
        double fy = networkEvent->conInfo()->fy;
        double cx = networkEvent->conInfo()->cx;
        double cy = networkEvent->conInfo()->cy;

        rtabmap::SensorData *sensorData = createSensorData(data, width, height, fx, fy, cx, cy);
        if (sensorData != NULL)
        {
            QCoreApplication::postEvent(_feature, new ImageEvent(sensorData, networkEvent->conInfo()));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(networkEvent->conInfo()));
        }
        return true;
    }
    return QObject::event(event);
}

rtabmap::SensorData *CameraNetwork::createSensorData(std::vector<unsigned char> *data, int width, int height, double fx, double fy, double cx, double cy)
{
    UDEBUG("");
    if (data != NULL)
    {
        UDEBUG("Received image width %d, height %d", width, height);
        // there is no data copy here, the cv::Mat has a pointer to the data
        cv::Mat img(height, width, CV_8UC1, &(*data)[0]);

        //imwrite("image.jpg", img);

        if (!img.empty())
        {
            rtabmap::Transform localTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
            rtabmap::CameraModel model(fx, fy, cx, cy, localTransform);
            rtabmap::SensorData *sensorData = new rtabmap::SensorData(img, model);
            img.release(); // decrement the reference counter
            return sensorData;
        }
    }

    return NULL;
}
