#pragma once

#include <rtabmap/core/Camera.h>
#include <QObject>
#include "Localization.h"
#include "HTTPServer.h"

#define WIDTH 640
#define HEIGHT 480

class Localization;
class HTTPServer;

class CameraNetwork :
    public QObject
{
public:
    CameraNetwork(const rtabmap::Transform &localTransform);
    virtual ~CameraNetwork();

    bool init(const std::string &calibrationFolder, const std::string &cameraName);

    void setLocalizer(Localization *loc);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    rtabmap::SensorData *process(std::vector<unsigned char> *data);
    static cv::Mat dataToImage(std::vector<unsigned char> *data);

private:
    rtabmap::CameraModel _model;
    Localization *_loc;
    HTTPServer *_httpServer;
};
