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

    void setLocalization(Localization *loc);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    rtabmap::SensorData *createSensorData(std::vector<unsigned char> *data, int width, int height, double fx, double fy, double cx, double cy);

private:
    Localization *_loc;
    HTTPServer *_httpServer;
};
