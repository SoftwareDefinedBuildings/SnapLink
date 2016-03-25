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

        std::vector<unsigned char> widthBytes = networkEvent->conInfo()->width;
        std::vector<unsigned char> heightBytes = networkEvent->conInfo()->height;
        if (widthBytes.size() != 4 || heightBytes.size() != 4) {
            return false;
        }

        // convert into integers (vector bytes is in network order)
        int width, height;
        width = (widthBytes[0] << 24) | (widthBytes[1]) << 16 | (widthBytes[2] << 8) | (widthBytes[3]);
        height = (heightBytes[0] << 24) | (heightBytes[1]) << 16 | (heightBytes[2] << 8) | (heightBytes[3]);

        rtabmap::SensorData *sensorData = process(&networkEvent->conInfo()->data, width, height);
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

rtabmap::SensorData *CameraNetwork::process(std::vector<unsigned char> *data, int width, int height)
{
    UDEBUG("");
    if (data != NULL)
    {
        // there is no data copy here, the cv::Mat has a pointer to the data
        cv::Mat img(height, width, CV_8UC1, &(*data)[0]);
        cv::flip(img, img, 0); // flip the image around the x-axis

        //imwrite("image.jpg", img);

        if (!img.empty())
        {
            rtabmap::SensorData *sensorData = new rtabmap::SensorData(img, _model);
            img.release(); // decrement the reference counter
            return sensorData;
        }
    }

    return NULL;
}
