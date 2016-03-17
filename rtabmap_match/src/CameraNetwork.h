#pragma once

#include <rtabmap/core/Camera.h>
#include <QObject>

#include "Localization.h"

class Localization;

class CameraNetwork :
    public QObject
{
public:
    CameraNetwork(bool rectifyImages = false,
                  bool isDepth = false,
                  float imageRate = 0,
                  const rtabmap::Transform &localTransform = rtabmap::Transform::getIdentity());
    virtual ~CameraNetwork();

    virtual bool init(const std::string &calibrationFolder = ".", const std::string &cameraName = "");

    void setLocalizer(Localization *localizer);

protected:
    virtual rtabmap::SensorData captureImage();

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    bool addImage(std::vector<unsigned char> *data);
    static cv::Mat dataToImage(std::vector<unsigned char> *data);

private:
    cv::Mat _img;
    bool _rectifyImages;
    bool _isDepth;

    std::string _cameraName;
    rtabmap::CameraModel _model;
    Localization *_localizer;
};
