#pragma once

#include <rtabmap/core/Camera.h>
#include <QObject>
#include "Localization.h"
#include "HTTPServer.h"

class Localization;
class HTTPServer;

class CameraNetwork :
    public QObject
{
public:
    CameraNetwork();
    virtual ~CameraNetwork();

    bool init(const rtabmap::Transform &localTransform, const std::string &calibrationFolder, const std::string &cameraName);

    void setLocalizer(Localization *loc);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    rtabmap::SensorData *process(std::vector<unsigned char> *data, int width, int height);

private:
    rtabmap::CameraModel _model;
    Localization *_loc;
    HTTPServer *_httpServer;
};
