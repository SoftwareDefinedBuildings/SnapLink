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

    void setLocalizer(Localization *loc);
    void setHTTPServer(HTTPServer *httpServer);

protected:
    virtual bool event(QEvent *event);

private:
    // ownership transferred
    rtabmap::SensorData *createSensorData(std::vector<unsigned char> *data, uint32_t width, uint32_t height);

private:
    Localization *_loc;
    HTTPServer *_httpServer;
};
