#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>
#include <QCoreApplication>

#include "CameraNetwork.h"
#include "NetworkEvent.h"
#include "ImageEvent.h"

CameraNetwork::CameraNetwork(const rtabmap::Transform &localTransform, const std::string &calibrationFolder, const std::string &cameraName)
{
    UDEBUG("");

    _model.setLocalTransform(localTransform);

    // look for calibration files
    if (!calibrationFolder.empty() && !cameraName.empty())
    {
        if (!_model.load(calibrationFolder, cameraName))
        {
            UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!", cameraName.c_str(), calibrationFolder.c_str());
        }
        else
        {
            UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
                  _model.fx(),
                  _model.fy(),
                  _model.cx(),
                  _model.cy());
        }
    }
}

CameraNetwork::~CameraNetwork(void)
{
}

void CameraNetwork::setLocalizer(Localization *localizer)
{
    _localizer = localizer;
}

bool CameraNetwork::event(QEvent *event)
{
    if (event->type() == NetworkEvent::type())
    {
        NetworkEvent *networkEvent = static_cast<NetworkEvent *>(event);
        const rtabmap::SensorData *sensorData = process(networkEvent->payload());
        if (sensorData != NULL)
        {
            QCoreApplication::postEvent(_localizer, new ImageEvent(sensorData, networkEvent->conInfo()));
        }
        // TODO send failure event to HTTPServer
        return true;
    }
    return QObject::event(event);
}

rtabmap::SensorData *CameraNetwork::process(std::vector<unsigned char> *data)
{
    UDEBUG("");
    if (data != NULL)
    {
        _img = dataToImage(data);

        //imwrite("image.jpg", _img);

        if (!_img.empty())
        {
            rtabmap::SensorData *sensorData = new rtabmap::SensorData(_img, _model);
            _img.release(); // decrement the reference counter
            return sensorData; // no need to check if it's NULL
        }
    }

    return NULL;
}


cv::Mat CameraNetwork::dataToImage(std::vector<unsigned char> *data)
{
    cv::Mat mat(HEIGHT, WIDTH, CV_8UC1, &(*data)[0]);
    // TODO: look at cv:Ptr so we don't need to copy
    mat = mat.clone(); // so we can free data later
    delete data;

    cv::flip(mat, mat, 0); // flip the image around the x-axis

    return mat;
}
