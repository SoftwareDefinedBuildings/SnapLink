#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <cstdio>
#include <opencv2/imgproc/imgproc.hpp>
#include <QCoreApplication>

#include "CameraNetwork.h"
#include "NetworkEvent.h"
#include "ImageEvent.h"

CameraNetwork::CameraNetwork(bool rectifyImages,
                             bool isDepth,
                             float imageRate,
                             const rtabmap::Transform &localTransform) :
    _rectifyImages(rectifyImages),
    _isDepth(isDepth)
{
    _model.setLocalTransform(localTransform);
}

CameraNetwork::~CameraNetwork(void)
{
}

bool CameraNetwork::init(const std::string &calibrationFolder, const std::string &cameraName)
{
    _cameraName = cameraName;

    UDEBUG("");

    // look for calibration files
    if (!calibrationFolder.empty() && !cameraName.empty())
    {
        if (!_model.load(calibrationFolder, cameraName))
        {
            UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
                  cameraName.c_str(), calibrationFolder.c_str());
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

    if (_rectifyImages && !_model.isValidForRectification())
    {
        UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
        return false;
    }

    return true;
}

void CameraNetwork::setLocalizer(Localization *localizer)
{
    _localizer = localizer;
}

rtabmap::SensorData CameraNetwork::captureImage()
{
    UDEBUG("");
    if (!_img.empty())
    {
        rtabmap::SensorData sensorData(_img, _model);
        _img.release(); // decrement the reference counter
        return sensorData;
    }
    else
    {
        return rtabmap::SensorData(); // return empty data
    }
}

bool CameraNetwork::event(QEvent *event)
{
    if (event->type() == NetworkEvent::type())
    {
        NetworkEvent *networkEvent = static_cast<NetworkEvent *>(event);
        addImage(const_cast<std::vector<unsigned char> *>(networkEvent->payload()));
        rtabmap::SensorData *sensorData = new rtabmap::SensorData();
        *sensorData = captureImage();
        QCoreApplication::postEvent(_localizer, new ImageEvent(sensorData, networkEvent->conInfo()));
        return true;
    }
    return QObject::event(event);
}

bool CameraNetwork::addImage(std::vector<unsigned char> *data)
{
    UDEBUG("");
    if (data == NULL)
    {
        return false;
    }

    _img = dataToImage(data);

    //imwrite("image.jpg", _img);

    delete data;

    return true;
}


cv::Mat CameraNetwork::dataToImage(std::vector<unsigned char> *data)
{
    // TODO: hardcoded for now
    int width = 640;
    int height = 480;

    cv::Mat mat(height, width, CV_8UC1, &(*data)[0]);
    // TODO: look at cv:Ptr so we don't need to copy
    mat = mat.clone(); // so we can free data later

    cv::flip(mat, mat, 0); // flip the image around the x-axis

    return mat;
}
