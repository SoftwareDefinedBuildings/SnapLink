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
    _loc(NULL),
    _httpServer(NULL)
{
}

CameraNetwork::~CameraNetwork()
{
    _loc = NULL;
    _httpServer = NULL;
}

bool CameraNetwork::init(const rtabmap::Transform &localTransform, const std::string &calibrationFolder, const std::string &cameraName)
{
    _model.setLocalTransform(localTransform);

    // look for calibration files
    if (!calibrationFolder.empty() && !cameraName.empty())
    {
        if (_model.load(calibrationFolder, cameraName))
        {
            UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
                  _model.fx(),
                  _model.fy(),
                  _model.cx(),
                  _model.cy());
            return true;
        }
        else
        {
            UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!", cameraName.c_str(), calibrationFolder.c_str());
        }
    }
    return false;
}

void CameraNetwork::setLocalizer(Localization *loc)
{
    _loc = loc;
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
        rtabmap::SensorData *sensorData = process(networkEvent->payload());
        if (sensorData != NULL)
        {
            QCoreApplication::postEvent(_loc, new ImageEvent(sensorData, networkEvent->conInfo()));
        }
        else
        {
            QCoreApplication::postEvent(_httpServer, new FailureEvent(networkEvent->conInfo()));
        }
        return true;
    }
    return QObject::event(event);
}

rtabmap::SensorData *CameraNetwork::process(std::vector<unsigned char> *data)
{
    UDEBUG("");
    if (data != NULL)
    {
        cv::Mat img = dataToImage(data);

        //imwrite("image.jpg", img);

        if (!img.empty())
        {
            rtabmap::SensorData *sensorData = new rtabmap::SensorData(img, _model);
            img.release(); // decrement the reference counter
            return sensorData; // no need to check if it's NULL
        }
    }

    return NULL;
}


cv::Mat CameraNetwork::dataToImage(std::vector<unsigned char> *data)
{
    cv::Mat mat(HEIGHT, WIDTH, CV_8UC1, &(*data)[0]);
    // TODO: there is a copy here
    mat = mat.clone(); // so we can free data later
    delete data;

    cv::flip(mat, mat, 0); // flip the image around the x-axis

    return mat;
}
